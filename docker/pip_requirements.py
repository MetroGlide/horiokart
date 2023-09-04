import os
import glob
import argparse
import subprocess


def pip_install(path, recursive):
    # search all requirements.txt
    if recursive:
        search_path = os.path.join(path, '**', 'requirements.txt')
    else:
        search_path = os.path.join(path, 'requirements.txt')

    files = glob.glob(search_path, recursive=True)

    if len(files) == 0:
        return

    # install requirements
    for file in files:
        print(f"Installing requirements from {file}")
        subprocess.run(['pip', 'install', '-r', file])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Import files using vcstool')
    parser.add_argument('-s', '--search-path', required=True,
                        help='Path to search for files')
    parser.add_argument('-r', action='store_true',
                        help='Recursively search for files')
    args = parser.parse_args()

    pip_install(args.search_path, args.r)
