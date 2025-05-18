#!/usr/bin/env python3

import ast
import sys
import os

def analyze_python_file(filepath):
    """
    Analyzes a Python file for missing docstrings in modules, classes,
    functions, and methods.

    Args:
        filepath (str): The path to the Python file.

    Returns:
        list: A list of strings, where each string describes a missing docstring.
              Returns an empty list if no missing docstrings are found or if the
              file cannot be processed.
    """
    missing_docstrings_report = []
    try:
        with open(filepath, "r", encoding="utf-8") as f:
            source_code = f.read()
        # If the file is empty, ast.parse will raise a SyntaxError
        if not source_code.strip():
            # missing_docstrings_report.append(f"File: {filepath}\n  Info: File is empty or contains only whitespace.")
            return [] # Treat as no missing docstrings for reporting purposes
    except FileNotFoundError:
        missing_docstrings_report.append(f"File: {filepath}\n  Error: File not found.")
        return missing_docstrings_report
    except Exception as e:
        missing_docstrings_report.append(f"File: {filepath}\n  Error: Could not read file - {e}")
        return missing_docstrings_report

    try:
        tree = ast.parse(source_code, filename=filepath)
    except SyntaxError as e:
        missing_docstrings_report.append(
            f"File: {filepath}\n  Error: Syntax error at line {e.lineno}, offset {e.offset}: {e.msg}"
        )
        return missing_docstrings_report
    except Exception as e: # Catch other potential parsing errors
        missing_docstrings_report.append(
            f"File: {filepath}\n  Error: Could not parse file - {e}"
        )
        return missing_docstrings_report


    file_specific_findings = []
    # 1. Check module docstring
    if not ast.get_docstring(tree, clean=False):
        file_specific_findings.append(
            f"  Line 1: Module - Missing module docstring."
        )

    # 2. Traverse for classes, functions, and methods
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef):
            if not ast.get_docstring(node, clean=False):
                file_specific_findings.append(
                    f"  Line {node.lineno}: Class `{node.name}` - Missing docstring."
                )
            # Check methods within this class
            for child_node in node.body:
                if isinstance(child_node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    if not ast.get_docstring(child_node, clean=False):
                        # Avoid double-reporting if already caught as a general function by a less specific check
                        method_sig = f"Method `{node.name}.{child_node.name}`"
                        already_reported_as_function = any(
                            f"Function `{child_node.name}`" in finding and f"Line {child_node.lineno}" in finding
                            for finding in file_specific_findings
                        )
                        if not already_reported_as_function:
                             file_specific_findings.append(
                                f"  Line {child_node.lineno}: {method_sig} - Missing docstring."
                            )

        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            # Check if it's a top-level function (not a method)
            # A function is top-level if its direct parent is an ast.Module
            # This requires knowing the parent. A simpler way for ast.walk is to check
            # if it's NOT a method (i.e., not a direct child of a ClassDef).
            is_method = False
            for parent_candidate in ast.walk(tree): # Re-walk to find parent; not most efficient but works
                if isinstance(parent_candidate, ast.ClassDef):
                    if node in parent_candidate.body:
                        is_method = True
                        break
            
            if not is_method and not ast.get_docstring(node, clean=False):
                 file_specific_findings.append(
                    f"  Line {node.lineno}: Function `{node.name}` - Missing docstring."
                )
    
    if file_specific_findings:
        # Prepend the filepath to each finding for this file
        missing_docstrings_report.append(f"File: {filepath}")
        missing_docstrings_report.extend(sorted(list(set(file_specific_findings)))) # Sort and unique within file

    return missing_docstrings_report


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python check_docstrings.py <path_to_python_file_or_directory>")
        sys.exit(1)

    target_path = sys.argv[1]
    all_reports = []
    files_processed_count = 0
    files_with_missing_docstrings_count = 0

    if os.path.isfile(target_path):
        if target_path.lower().endswith(".py"):
            print(f"\n--- Analyzing File: {target_path} ---")
            report = analyze_python_file(target_path)
            files_processed_count = 1
            if report: # report will contain at least the filename if findings exist
                all_reports.extend(report)
                if len(report) > 1: # More than just the filename entry
                     files_with_missing_docstrings_count = 1
        else:
            print(f"Skipping non-Python file: {target_path}")
    elif os.path.isdir(target_path):
        print(f"\n--- Analyzing Directory (Recursively): {target_path} ---")
        for root, _, files in os.walk(target_path):
            for file in files:
                if file.lower().endswith(".py"):
                    filepath = os.path.join(root, file)
                    print(f"Processing: {filepath}...")
                    report = analyze_python_file(filepath)
                    files_processed_count += 1
                    if report: # report will contain at least the filename if findings exist
                        all_reports.extend(report)
                        # A report for a file starts with "File: <filepath>"
                        # If there are more entries, it means missing docstrings were found.
                        if len(report) > 1 or (len(report) == 1 and "Error:" in report[0]): # Count files with actual findings or errors
                            files_with_missing_docstrings_count +=1 
    else:
        print(f"Error: Path '{target_path}' is not a valid file or directory.")
        sys.exit(1)

    if all_reports:
        print("\n\n--- Aggregate Docstring Report ---")
        current_file_header = None
        for finding_line in all_reports:
            if finding_line.startswith("File: "):
                if current_file_header: # Add a small separator between files
                    print("-" * 20)
                current_file_header = finding_line
                print(f"\n{current_file_header}")
            else:
                print(finding_line) # This is already indented by analyze_python_file
        print("-" * 40)

    print(f"\n--- Summary ---")
    print(f"Total Python files processed: {files_processed_count}")
    print(f"Files with missing docstrings or errors: {files_with_missing_docstrings_count}")

    if files_processed_count == 0:
        print("No Python files found to analyze.")
