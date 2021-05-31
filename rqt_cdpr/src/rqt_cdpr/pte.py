#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# tree.py
#
# Written by Doug Dahms
#
# Prints the tree structure for the path specified on the command line

from os import listdir, sep
from os.path import abspath, basename, isdir
from sys import argv

def tree(dir, padding, print_files=False, isLast=False, isFirst=False):
    if isFirst:
        print padding.decode('utf8')[:-1].encode('utf8') + dir
    else:
        if isLast:
            print padding.decode('utf8')[:-1].encode('utf8') + '└── ' + basename(abspath(dir))
        else:
            print padding.decode('utf8')[:-1].encode('utf8') + '├── ' + basename(abspath(dir))
    files = []
    if print_files:
        files = listdir(dir)
    else:
        files = [x for x in listdir(dir) if isdir(dir + sep + x)]
    if not isFirst:
        padding = padding + '   '
    files = sorted(files, key=lambda s: s.lower())
    count = 0
    last = len(files) - 1
    for i, file in enumerate(files):
        count += 1
        path = dir + sep + file
        isLast = i == last
        if isdir(path):
            if count == len(files):
                if isFirst:
                    tree(path, padding, print_files, isLast, False)
                else:
                    tree(path, padding + ' ', print_files, isLast, False)
            else:
                tree(path, padding + '│', print_files, isLast, False)
        else:
            if isLast:
                print padding + '└── ' + file
            else:
                print padding + '├── ' + file

def usage():
    return '''Usage: %s [-f]
Print tree structure of path specified.
Options:
-f      Print files as well as directories
PATH    Path to process''' % basename(argv[0])

def main():
    if len(argv) == 1:
        print usage()
    elif len(argv) == 2:
        # print just directories
        path = argv[1]
        if isdir(path):
            tree(path, '', False, False, True)
        else:
            print 'ERROR: \'' + path + '\' is not a directory'
    elif len(argv) == 3 and argv[1] == '-f':
        # print directories and files
        path = argv[2]
        if isdir(path):
            tree(path, '', True, False, True)
        else:
            print 'ERROR: \'' + path + '\' is not a directory'
    else:
        print usage()

if __name__ == '__main__':
    main()