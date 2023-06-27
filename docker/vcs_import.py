import os
import argparse
import subprocess


DONE_PATH_LIST = []

def import_files(path, recursive, output_path):
    # 指定したパスでファイルを列挙する
    files = []
    if recursive:
        for root, _, filenames in os.walk(path):
            for filename in filenames:
                if filename not in DONE_PATH_LIST:
                    DONE_PATH_LIST.append(filename)
                    files.append(os.path.join(root, filename))
    else:
        for filename in os.listdir(path):
            if filename not in DONE_PATH_LIST:
                DONE_PATH_LIST.append(filename)
                files.append(os.path.join(path, filename))
    
    if len(files) == 0:
        return False

    # 拡張子が .repos もしくは .rosinstall のファイルに対して vcs import を実行する
    for file in files:
        _, extension = os.path.splitext(file)
        if extension == '.repos' or extension == '.rosinstall':
            # vcs import output_path < file
            print(f"vcs import {output_path} < {file}")
            command = ['vcs', 'import', output_path]
            subprocess.run(command, stdin=open(file, 'r'))
    return True

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Import files using vcstool')
    parser.add_argument('-s', '--search-path', required=True, help='Path to search for files')
    parser.add_argument('-r', action='store_true', help='Recursively search for files')
    parser.add_argument('-o', '--output-path', required=True, help='Download path for vcstool')
    args = parser.parse_args()

    import_files(args.search_path, args.r, args.output_path)

    while import_files(args.output_path, args.r, args.output_path):
        pass
