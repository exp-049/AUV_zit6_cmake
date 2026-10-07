#!/usr/bin/env python3
"""Cache the generated micro-ROS archive without discarding a working library.

An existing library is trusted only after this entrypoint generated it, or
after an operator explicitly runs --mark-verified-library following validation.
"""

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


LIBRARY_FOLDER = Path(
    'UserApp/Thirdparty/micro_ros_stm32cubemx_utils/microros_static_library_ide')
REPOS_TEMPLATE = Path(
    'UserApp/Thirdparty/micro_ros_stm32cubemx_utils/microros_static_library/'
    'library_generation/extra_packages/extra_packages.repos')
STAMP_NAME = '.generation-cache.json'
DEFAULT_IMAGE = 'microros/micro_ros_static_library_builder:humble'
IGNORED_PARTS = {'.git', '__pycache__', '.pytest_cache', 'build', 'install', 'log'}


def interface_files(root):
    return sorted(path for path in (root / 'zit6_interfaces').rglob('*')
                  if path.is_file() and not (set(path.parts) & IGNORED_PARTS)
                  and (path.suffix in ('.msg', '.srv', '.action')
                       or path.name in ('CMakeLists.txt', 'package.xml')))


def input_fingerprint(root, image):
    generator = root / LIBRARY_FOLDER / 'library_generation'
    inputs = interface_files(root)
    if not inputs:
        raise ValueError('zit6_interfaces sources are missing')
    inputs += [generator / name for name in (
        'library_generation.sh', 'colcon.meta', 'colcon-embeddedrtps.meta',
        'toolchain.cmake', 'extract_flags.py')]
    inputs += [root / REPOS_TEMPLATE, root / 'UserApp/Config/microros_config.meta',
               root / 'scripts/merge_microros_config.py']
    digest = hashlib.sha256()
    settings = {'version': 1, 'image': image,
                'embeddedrtps': os.environ.get('MICROROS_USE_EMBEDDEDRTPS')}
    digest.update(json.dumps(settings, sort_keys=True).encode())
    for path in sorted(inputs):
        digest.update(str(path.relative_to(root)).encode() + b'\0')
        digest.update(path.read_bytes() if path.is_file() else b'<absent>')
        digest.update(b'\0')
    digest.update(Path(__file__).read_bytes())
    return digest.hexdigest()


def library_digest(directory):
    archive = directory / 'libmicroros.a'
    if not archive.is_file():
        raise ValueError('generated libmicroros.a is missing')
    with archive.open('rb') as stream:
        if stream.read(8) != b'!<arch>\n' or archive.stat().st_size <= 8:
            raise ValueError('generated libmicroros.a is not a nonempty archive')
    if not any((directory / 'include').rglob('*.h')):
        raise ValueError('generated micro-ROS headers are missing')
    digest = hashlib.sha256()
    for path in sorted(directory.rglob('*')):
        if path.is_file() and path.name != STAMP_NAME:
            digest.update(str(path.relative_to(directory)).encode() + b'\0')
            digest.update(path.read_bytes())
            digest.update(b'\0')
    return digest.hexdigest()


def write_stamp(directory, fingerprint):
    record = {'input_sha256': fingerprint, 'artifact_sha256': library_digest(directory)}
    with tempfile.NamedTemporaryFile(mode='w', dir=directory, delete=False) as stream:
        json.dump(record, stream, sort_keys=True)
        stream.write('\n')
        temporary = stream.name
    os.replace(temporary, directory / STAMP_NAME)


def cache_matches(directory, fingerprint):
    try:
        record = json.loads((directory / STAMP_NAME).read_text())
        return (record['input_sha256'] == fingerprint
                and record['artifact_sha256'] == library_digest(directory))
    except (OSError, ValueError, KeyError):
        return False


def generate(root, image, docker, fingerprint):
    scratch = Path(tempfile.mkdtemp(prefix='auv-microros-', dir='/tmp'))
    destination = root / LIBRARY_FOLDER / 'libmicroros'
    try:
        staged_base = scratch / LIBRARY_FOLDER
        staged_generator = staged_base / 'library_generation'
        shutil.copytree(root / LIBRARY_FOLDER / 'library_generation', staged_generator,
                        ignore=shutil.ignore_patterns('extra_packages', '__pycache__'))
        extra = staged_generator / 'extra_packages'
        extra.mkdir()
        # Copy only interface package inputs, never colcon build/install caches.
        for path in interface_files(root):
            copied = extra / 'zit6_interfaces' / path.relative_to(root / 'zit6_interfaces')
            copied.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, copied)
        shutil.copy2(root / REPOS_TEMPLATE, extra / 'extra_packages.repos')
        config = root / 'UserApp/Config/microros_config.meta'
        if config.is_file():
            subprocess.run([sys.executable, str(root / 'scripts/merge_microros_config.py'),
                            str(staged_generator / 'colcon.meta'), str(config),
                            str(staged_generator / 'colcon.meta')], check=True)
        command = [docker, 'run', '--rm', '--network', 'host', '-v',
                   f'{scratch}:/project', '--env', f'MICROROS_LIBRARY_FOLDER={LIBRARY_FOLDER}']
        for name, default in (('http_proxy', 'http://127.0.0.1:7897'),
                              ('https_proxy', 'http://127.0.0.1:7897'),
                              ('all_proxy', 'socks5://127.0.0.1:7897')):
            command += ['--env', f'{name}={os.environ.get(name, default)}']
        if 'MICROROS_USE_EMBEDDEDRTPS' in os.environ:
            command += ['--env', 'MICROROS_USE_EMBEDDEDRTPS=' +
                        os.environ['MICROROS_USE_EMBEDDEDRTPS']]
        subprocess.run(command + [image], check=True)
        generated = staged_base / 'libmicroros'
        library_digest(generated)
        if input_fingerprint(root, image) != fingerprint:
            raise ValueError('micro-ROS inputs changed during generation; keeping previous library')
        # Prepare on the destination filesystem before replacing the directory.
        # If generation/copy fails, the previous library is still untouched.
        with tempfile.TemporaryDirectory(prefix='.microros-install-',
                                         dir=destination.parent) as install:
            install = Path(install)
            replacement, previous = install / 'new', install / 'previous'
            shutil.copytree(generated, replacement)
            write_stamp(replacement, fingerprint)
            had_previous = destination.exists()
            if had_previous:
                os.replace(destination, previous)
            try:
                os.replace(replacement, destination)
            except OSError:
                if had_previous:
                    os.replace(previous, destination)
                raise
        print('micro-ROS library regenerated and installed')
    finally:
        shutil.rmtree(scratch, ignore_errors=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path,
                        default=Path(__file__).resolve().parents[1])
    parser.add_argument('--docker', default='docker')
    parser.add_argument('--image', default=DEFAULT_IMAGE)
    parser.add_argument('--mark-verified-library', action='store_true',
                        help='explicitly attest the current library was built and verified '
                             'with these inputs; never inferred automatically')
    args = parser.parse_args(argv)
    root = args.project_root.resolve()
    directory = root / LIBRARY_FOLDER / 'libmicroros'
    directory.parent.mkdir(parents=True, exist_ok=True)
    with (directory.parent / 'libmicroros.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        fingerprint = input_fingerprint(root, args.image)
        if args.mark_verified_library:
            write_stamp(directory, fingerprint)
            print('Explicitly marked the verified current micro-ROS library')
        elif cache_matches(directory, fingerprint):
            print('micro-ROS input and artifact fingerprints match; skipping Docker')
        else:
            generate(root, args.image, args.docker, fingerprint)
        # CMake's real archive output must become newer than unchanged inputs
        # that were touched/re-copied, without forcing another Docker invocation.
        (directory / 'libmicroros.a').touch()
        (directory / STAMP_NAME).touch()


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f'micro-ROS library generation failed: {error}', file=sys.stderr)
        sys.exit(1)
