#!/usr/bin/env python3
"""Exercise the micro-ROS cache and replacement path with a fake Docker CLI."""

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


PROJECT = Path(__file__).resolve().parents[1]
LIBRARY_FOLDER = Path(
    'UserApp/Thirdparty/micro_ros_stm32cubemx_utils/microros_static_library_ide')


class MicroRosLibraryCacheTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='microros-cache-test-')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / 'project'
        self.root.mkdir()

        for relative in ('scripts/ensure_microros_library.py',
                         'scripts/merge_microros_config.py'):
            destination = self.root / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(PROJECT / relative, destination)

        interfaces = self.root / 'zit6_interfaces'
        (interfaces / 'msg').mkdir(parents=True)
        (interfaces / 'msg/Probe.msg').write_text('uint32 value\n')
        (interfaces / 'srv').mkdir()
        (interfaces / 'srv/Probe.srv').write_text('---\nbool success\n')
        (interfaces / 'CMakeLists.txt').write_text('project(zit6_interfaces)\n')
        (interfaces / 'package.xml').write_text('<package/>\n')

        generation = (self.root / LIBRARY_FOLDER / 'library_generation')
        generation.mkdir(parents=True)
        for name, content in {
            'library_generation.sh': '#!/bin/bash\n',
            'colcon.meta': '{"names": {}}\n',
            'colcon-embeddedrtps.meta': '{}\n',
            'toolchain.cmake': 'set(CMAKE_SYSTEM_NAME Generic)\n',
            'extract_flags.py': '# placeholder\n',
        }.items():
            (generation / name).write_text(content)

        repos = (self.root / 'UserApp/Thirdparty/micro_ros_stm32cubemx_utils/'
                 'microros_static_library/library_generation/extra_packages')
        repos.mkdir(parents=True)
        (repos / 'extra_packages.repos').write_text('repositories: {}\n')
        config = self.root / 'UserApp/Config/microros_config.meta'
        config.parent.mkdir(parents=True)
        config.write_text(json.dumps({'rmw_microxrcedds': {
            'RMW_UXRCE_MAX_SERVICES': 4}}))

        library = self.root / LIBRARY_FOLDER / 'libmicroros'
        (library / 'include').mkdir(parents=True)
        (library / 'libmicroros.a').write_bytes(b'!<arch>\noriginal')
        (library / 'include/original.h').write_text('/* original */\n')

        self.fake_docker = Path(self.temporary.name) / 'fake-docker'
        self.fake_docker.write_text(
            '#!/usr/bin/env python3\n'
            'import os, pathlib, sys\n'
            'if os.environ.get("FAKE_DOCKER_FAIL"): sys.exit(17)\n'
            'args = sys.argv[1:]\n'
            'mount = args[args.index("-v") + 1].split(":", 1)[0]\n'
            'item = args[args.index("--env") + 1]\n'
            'folder = item.split("=", 1)[1]\n'
            'out = pathlib.Path(mount) / folder / "libmicroros"\n'
            '(out / "include").mkdir(parents=True)\n'
            '(out / "libmicroros.a").write_bytes(b"!<arch>\\ngenerated")\n'
            '(out / "include/generated.h").write_text("/* generated */\\n")\n')
        self.fake_docker.chmod(0o755)

    def run_helper(self, *arguments, fail_docker=False):
        env = os.environ.copy()
        if fail_docker:
            env['FAKE_DOCKER_FAIL'] = '1'
        else:
            env.pop('FAKE_DOCKER_FAIL', None)
        return subprocess.run(
            [sys.executable, str(self.root / 'scripts/ensure_microros_library.py'),
             '--project-root', str(self.root), '--docker', str(self.fake_docker),
             *arguments], text=True, capture_output=True, env=env, check=False)

    def mark_current_library_verified(self):
        result = self.run_helper('--mark-verified-library')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_verified_cache_hit_does_not_call_docker(self):
        self.mark_current_library_verified()

        result = self.run_helper(fail_docker=True)

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('skipping Docker', result.stdout)
        archive = self.root / LIBRARY_FOLDER / 'libmicroros/libmicroros.a'
        self.assertEqual(archive.read_bytes(), b'!<arch>\noriginal')

    def test_changed_interfaces_regenerate_and_cache_new_library(self):
        self.mark_current_library_verified()
        interface = self.root / 'zit6_interfaces/msg/Probe.msg'
        interface.write_text('uint64 value\n')

        generated = self.run_helper()

        self.assertEqual(generated.returncode, 0, generated.stderr)
        self.assertIn('regenerated and installed', generated.stdout)
        library = self.root / LIBRARY_FOLDER / 'libmicroros'
        self.assertEqual((library / 'libmicroros.a').read_bytes(),
                         b'!<arch>\ngenerated')
        self.assertTrue((library / 'include/generated.h').is_file())
        cached = self.run_helper(fail_docker=True)
        self.assertEqual(cached.returncode, 0, cached.stderr)

    def test_generation_failure_keeps_previous_library_and_stamp(self):
        self.mark_current_library_verified()
        library = self.root / LIBRARY_FOLDER / 'libmicroros'
        before_archive = (library / 'libmicroros.a').read_bytes()
        before_header = (library / 'include/original.h').read_bytes()
        before_stamp = (library / '.generation-cache.json').read_bytes()
        (self.root / 'zit6_interfaces/srv/Probe.srv').write_text(
            'uint32 request\n---\nbool success\n')

        result = self.run_helper(fail_docker=True)

        self.assertNotEqual(result.returncode, 0)
        self.assertEqual((library / 'libmicroros.a').read_bytes(), before_archive)
        self.assertEqual((library / 'include/original.h').read_bytes(), before_header)
        self.assertEqual((library / '.generation-cache.json').read_bytes(), before_stamp)


if __name__ == '__main__':
    unittest.main()
