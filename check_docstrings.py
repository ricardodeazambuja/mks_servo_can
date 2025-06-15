import ast
import sys

def check_file_docstrings(filepath):
    try:
        with open(filepath, "r", encoding="utf-8") as source:
            tree = ast.parse(source.read(), filename=filepath)
    except Exception as e:
        # print(f"Error parsing {filepath}: {e}", file=sys.stderr) # Suppressing for cleaner output
        return False # Indicate error or inability to parse

    missing_docstrings_found = False
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef):
            if not ast.get_docstring(node):
                print(f"{filepath}:Class:{node.name}")
                missing_docstrings_found = True
            # Check methods within the class
            for sub_node in node.body:
                if isinstance(sub_node, (ast.FunctionDef, ast.AsyncFunctionDef)):  # Method or Async Method
                    if not ast.get_docstring(sub_node):
                        # Distinguish __init__ from other methods for clarity if needed
                        method_type = "Method" if not sub_node.name.startswith("__") else "SpecialMethod"
                        if isinstance(sub_node, ast.AsyncFunctionDef):
                             method_type = "AsyncMethod" if not sub_node.name.startswith("__") else "AsyncSpecialMethod"

                        # For this check, we only care about presence, not type distinction beyond class/function
                        print(f"{filepath}:Method:{node.name}.{sub_node.name}")
                        missing_docstrings_found = True
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            # Check if it's a top-level function (not a method)
            # A simpler way to check for top-level is to see if its parent is a Module node
            parent = getattr(node, 'parent', None) # Requires parent pointers, ast.walk doesn't add them
                                                    # Instead, we check if it's directly under a Module
                                                    # This requires a slightly different walk or check
            
            is_top_level = False
            for direct_child_node in tree.body: # Iterate over top-level nodes in the AST
                if direct_child_node == node:
                    is_top_level = True
                    break

            if is_top_level:
                func_type = "Function"
                if isinstance(node, ast.AsyncFunctionDef):
                    func_type = "AsyncFunction"
                if not ast.get_docstring(node):
                    print(f"{filepath}:{func_type}:{node.name}")
                    missing_docstrings_found = True
    return missing_docstrings_found

if __name__ == "__main__":
    if len(sys.argv) < 2:
        # print("Usage: python check_docstrings.py <file1.py> [file2.py ...]", file=sys.stderr)
        sys.exit(1)

    any_missing = False
    for filepath_arg in sys.argv[1:]:
        if check_file_docstrings(filepath_arg):
            any_missing = True

    # Optionally, exit with a status code if any are missing
    # if any_missing:
    #     sys.exit(1)
