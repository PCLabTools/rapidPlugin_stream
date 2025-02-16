#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File name: build.py
Author: PCLabTools
Created: 2025-02-08
Version: 1.0
Description: This script runs automatically when running PlatformIO "build", run manually if using this library from a different IDE
"""

# Running in PlatformIO
try:
  Import("env") # type: ignore
  current_env = env["PIOENV"] # type: ignore
  project_path = env["PROJECT_DIR"] #type: ignore
  error = '\033[0;31m'
  warning = '\033[0;33m'
  report = '\033[0;32m'
  info = '\033[0;30m'

# Running in Isolation
except:
  import tkinter as tk
  from tkinter import filedialog

  def show_file_prompt():
      root = tk.Tk()
      root.withdraw()  # Hide the root window
      project_path = filedialog.askopenfilename()
      if project_path:
          print(f"Selected project path: {project_path}")
      else:
          print("No project path selected")

  show_file_prompt()
  error = '[error] '
  warning = '[warning] '
  report = ''
  info = '[info] '

print(report + 'running rapidPlugin_stream \'build.py\'...')

try:
  exec(open('modify_errors.py').read())

  print(report + 'rapidPlugin_stream \'build.py\' complete')

except:
  print(error + 'rapidPlugin_stream \'build.py\' failed')
  raise
