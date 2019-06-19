#!/usr/bin/env python3

import argparse
import multiprocessing
import os
import queue
import shlex
import subprocess
import sys
import threading

def make_tidy_command(file, clang_tidy_binary, checks, header_filter, cxxflags):
    command = "{binary} -header-filter={headers} -checks={checks} -warnings-as-errors={checks} -quiet {file} -- {cxxflags}".format(
        binary   = clang_tidy_binary,
        headers  = header_filter,
        checks   = checks,
        file     = file,
        cxxflags = cxxflags,
    )

    return shlex.split(command)


def run_tidy(args, task_queue, lock, error_queue, failed_files):
    # Loop until queue is empty
    while True:
        # Dequeue
        file = task_queue.get()

        # Determine command
        command = make_tidy_command(file, args.clang_tidy_binary, args.checks,
                                    args.header_filter, args.cxxflags)

        # Run command
        try:
            process = subprocess.Popen(command, stderr=subprocess.PIPE)
            _, err = process.communicate()
        except Exception as e:
            with lock:
                error_queue.put("Error invoking clang tidy : {}".format(e))
                failed_files.append(file)

            with task_queue.mutex:
                task_queue.queue.clear()
                task_queue.all_tasks_done.notify_all()
                task_queue.unfinished_tasks = 0

            return

        # Mark as failed
        if process.returncode != 0:
            with lock:
                failed_files.append(file)

        # Print stderr (which is stdout from clang tidy)
        if err:
            sys.stderr.write("{}: {}".format(file, err.decode('utf-8')))

        # Mark as done
        task_queue.task_done()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--checks', required=True)
    parser.add_argument('--clang-tidy-binary')
    parser.add_argument('--cxxflags', required=True)
    parser.add_argument('--header-filter', required=True)
    parser.add_argument('files', nargs='*')
    return parser.parse_args()


def spin(args):
    NUM_THREADS = multiprocessing.cpu_count()

    # Queue of tasks / Shared objects
    task_queue = queue.Queue(NUM_THREADS)
    error_queue = queue.Queue()
    failed_files = []
    lock = threading.Lock()

    # Spin threads
    for _ in range(NUM_THREADS):
        t = threading.Thread(target=run_tidy, args=(args, task_queue, lock, error_queue, failed_files))
        t.daemon = True
        t.start()

    # Fill the queue with files
    for name in args.files:
        task_queue.put(name)

    # Wait for all threads to be done.
    task_queue.join()

    # Check for failures
    if not error_queue.empty():
        print("Errors:")
        while not error_queue.empty():
            print(" ", error_queue.get())
        sys.exit(1)
    if failed_files:
        sys.exit(1)

def main():
    # Parse args
    args = parse_args()

    # Spin threads
    try:
        spin(args)
    except KeyboardInterrupt:
        os.kill(0, 9)

    sys.exit(0)


if __name__ == '__main__':
    main()
