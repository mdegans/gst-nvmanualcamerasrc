"""
Script to get the Argus version (kinda) because:
https://forums.developer.nvidia.com/t/argus-changelog/172781
"""

import subprocess
import sys
import os

def argus_version():
    """
    Returns:
        (Optional[str]) Argus apt package version or None if not available
    """
    try:
        cp = subprocess.run(
            ('apt-cache', 'policy', 'nvidia-l4t-jetson-multimedia-api'),
            stdout=subprocess.PIPE)
        cp.check_returncode()
        for line in cp.stdout.decode().split(os.linesep):
            # this will break on non-english systems, but fixing it not possible
            # for every language. The proper "fix" is to fix Argus versioning.
            if 'Installed:' in line:
                split_line = line.split('Installed:')
                version = split_line[-1].strip()
                if version:
                    return version
    except Exception as e:
        sys.stderr.write(f'{e}\n')
    return None

if __name__ == "__main__":
    print(argus_version())
