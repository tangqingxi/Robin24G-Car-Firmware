#!/usr/bin/env python3
"""
romlib_gen.py - convert romlib.def to romlib.h and romlib.inc files

This script processes the romlib.def file, wraps function names, and saves the result to romlib.h
For special variables, it also generates .equ directives in romlib.inc
Python version of romlib_gen.sh script
"""

import os
import re
import sys


def parse_symbol_table(list_file):
    """
    Parse symbol table file to extract symbol name to address mapping

    Args:
        list_file (str): symbol table file path

    Returns:
        dict: symbol name to address mapping dictionary
    """
    symbol_map = {}

    try:
        with open(list_file, 'r', encoding='utf-8', errors='ignore') as f:
            list_symbols = f.read()

        # Process each line to extract symbols
        for line in list_symbols.split('\n'):
            line = line.strip()
            if not line:
                continue

            # Match symbol table line format: address g [type] .section size symbol_name
            # Example: 00017010 g     F .rom_code	000000c8 ble_irq_handler
            # Or: 00841300 g       .data	00000000 _start_data_
            match = re.match(r'^([0-9a-fA-F]+)\s+g\s+.*?\s+(\w+)$', line)
            if match:
                address = match.group(1)
                symbol_name = match.group(2)
                symbol_map[symbol_name] = address

    except FileNotFoundError:
        print(f"Error: file {list_file} does not exist")
        sys.exit(1)
    except Exception as e:
        print(f"Error reading file {list_file}: {e}")
        sys.exit(1)

    return symbol_map


def extract_symbol_name(line):
    """
    Extract symbol name from line

    Args:
        line (str): input line

    Returns:
        tuple: (symbol_name, type) where type is 0=none, 1=function, 2=variable
    """
    line = line.strip()
    if not line:
        return "", 0

    # Check if it's a function declaration (contains '(')
    if '(' in line:
        # Extract function name: find the last word before '('
        before_paren = line.split('(')[0]
        words = before_paren.split()
        if words:
            return words[-1], 1
    # Check if it's a variable declaration (contains ';')
    elif ';' in line:
        # Extract variable name: find the last word before ';', remove pointer symbol if present
        before_semicolon = line.split(';')[0]
        words = before_semicolon.split()
        if words:
            var_name = words[-1]
            # Remove leading '*' if present
            var_name = var_name.lstrip('*')
            # Remove array brackets if present (e.g., 'smp_confirm[]' -> 'smp_confirm')
            var_name = re.sub(r'\[.*\]$', '', var_name)
            return var_name, 2

    return "", 0


def find_matching_line(symbol_name, list_symbols):
    """
    Find matching line for symbol with improved precision

    Args:
        symbol_name (str): symbol name to find
        list_symbols (str): all symbol lines

    Returns:
        str: matching line or empty string
    """
    # First try precise match: ensure symbol is at end of line or followed by space
    pattern = rf'^[0-9a-fA-F]+\s+.*\s+{re.escape(symbol_name)}(\s|$)'
    for line in list_symbols.split('\n'):
        if re.match(pattern, line):
            return line

    # If precise match fails, try simple match for backward compatibility
    for line in list_symbols.split('\n'):
        if symbol_name in line:
            return line

    return ""


def generate_function_declaration(line, symbol_name, address):
    """
    Generate function declaration

    Args:
        line (str): original line
        symbol_name (str): function name
        address (str): address

    Returns:
        tuple: (declaration, define_macro)
    """
    # Replace the rightmost ')' with ') = (void*)0xaddress'
    modified_line = re.sub(r'\)([^)]*)$', f') = (void*)0x{address}', line)

    # Wrap function name
    modified_function_name = f"(*p_{symbol_name})"

    # Replace original function name with full word match
    modified_line = re.sub(rf'\b{re.escape(symbol_name)}\b', modified_function_name, modified_line)

    # Add static const prefix and semicolon
    modified_line = f"static const {modified_line};"

    # Generate define macro
    define_line = f"#define {symbol_name}  {modified_function_name}"

    return modified_line, define_line


def generate_variable_declaration(line, symbol_name, address):
    """
    Generate variable declaration with improved handling for pointer types

    Args:
        line (str): original line
        symbol_name (str): variable name
        address (str): address

    Returns:
        tuple: (declaration, is_special_var) where is_special_var indicates if it's a startup variable
    """
    special_vars = ['_start_data_', '_end_data_', '_start_bss_', '_end_bss_', '_dstored_']

    if symbol_name in special_vars:
        # Generate .equ directive for special variables
        var_name = f"rom_{symbol_name}"
        modified_line = f".equ {var_name},  0x{address}"
        return modified_line, True

    # Check if it's an array type (contains '[' and ']' after the variable name)
    if re.search(r'\b' + re.escape(symbol_name) + r'\s*\[.*?\]', line):
        # Array variable: convert to single pointer type
        # Remove array dimension and convert to pointer
        type_decl = re.sub(r';.*$', '', line)
        type_decl = re.sub(r'\s*' + re.escape(symbol_name) + r'\s*\[.*?\]\s*$', '', type_decl).strip()
        # Generate: #define var_name ((type *)0xaddress)
        modified_line = f"#define {symbol_name} (({type_decl} *)0x{address})"

    # Check if it's a pointer type (contains '*' before the variable name)
    elif f'*{symbol_name}' in line or f'* {symbol_name}' in line:
        # Pointer variable: extract type declaration
        # Remove ';' and extract type (everything before '* symbol_name')
        type_decl = re.sub(r';.*$', '', line)
        type_decl = re.sub(r'\*\s*' + re.escape(symbol_name) + r'\s*$', '', type_decl).strip()
        # Generate: #define var_name (*(type **)0xaddress)
        modified_line = f"#define {symbol_name} (*({type_decl} **)0x{address})"
    else:
        # Non-pointer variable: standard handling
        modified_line = re.sub(r';([^;]*)$', ')', line)
        address_part = f"*)0x{address}"

        # Replace variable name with word boundary match first, fallback to simple replacement
        if re.search(rf'\b{re.escape(symbol_name)}\b', modified_line):
            modified_line = re.sub(rf'\b{re.escape(symbol_name)}\b', address_part, modified_line)
        else:
            modified_line = modified_line.replace(symbol_name, address_part)

        modified_line = f"#define {symbol_name} (*({modified_line}"

    return modified_line, False


def process_romlib_def(input_file, output_file, output_cstartup_file, symbol_map, list_symbols):
    """
    Process romlib.def file and generate romlib.h and romlib.inc

    Args:
        input_file (str): input file path
        output_file (str): output header file path
        output_cstartup_file (str): output cstartup file path (None if not needed,
                                    empty string for default romlib.inc path)
        symbol_map (dict): symbol mapping dictionary
        list_symbols (str): all symbol lines for precise matching
    """
    try:
        # Resolve default cstartup file path
        if output_cstartup_file == "":
            script_dir = os.path.dirname(os.path.abspath(__file__))
            output_cstartup_file = os.path.join(script_dir, "romlib.inc")

        # Clear output files
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write("#pragma once\n")

        if output_cstartup_file is not None:
            with open(output_cstartup_file, 'w', encoding='utf-8') as f:
                pass

        # Track processed symbols to avoid duplicates
        processed_symbols = set()

        with open(input_file, 'r', encoding='utf-8') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                if not line:
                    continue

                symbol_name, symbol_type = extract_symbol_name(line)

                if symbol_name:
                    # Use improved matching logic
                    matching_line = find_matching_line(symbol_name, list_symbols)

                    if matching_line:
                        # Skip if already processed
                        if symbol_name in processed_symbols:
                            print(f"Warning: line {line_num}, symbol '{symbol_name}' already processed, skipping duplicate")
                            continue

                        address = matching_line.split()[0]
                        processed_symbols.add(symbol_name)

                        if symbol_type == 1:  # function
                            func_decl, define_macro = generate_function_declaration(
                                line, symbol_name, address
                            )
                            with open(output_file, 'a', encoding='utf-8') as out_f:
                                out_f.write(f"{func_decl}\n")
                                out_f.write(f"{define_macro}\n")
                        elif symbol_type == 2:  # variable
                            var_decl, is_special = generate_variable_declaration(
                                line, symbol_name, address
                            )
                            if is_special and output_cstartup_file is not None:
                                # Write special variables (.equ) to cstartup/inc file
                                with open(output_cstartup_file, 'a', encoding='utf-8') as out_f:
                                    out_f.write(f"{var_decl}\n")
                            else:
                                # Write to header file
                                with open(output_file, 'a', encoding='utf-8') as out_f:
                                    out_f.write(f"{var_decl}\n")
                    else:
                        print(f"Warning: line {line_num}, no matching symbol found for: {symbol_name}")

        #print(f"Processing complete, results saved to {output_file}")
        #print(f"Total symbols processed: {len(processed_symbols)}")

    except FileNotFoundError:
        print(f"Error: file {input_file} does not exist")
        sys.exit(1)
    except Exception as e:
        print(f"Error processing files: {e}")
        sys.exit(1)


def main():
    """Main function"""
    # File path configuration
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_file = os.path.join(script_dir, "romlib.def")
    output_file = os.path.join(script_dir, "romlib.h")
    list_file = os.path.join(script_dir, "rom_lib.lst")

    print("Starting romlib.h and romlib.inc generation...")

    # Parse symbol table
    #print(f"Reading symbol table file: {list_file}")
    symbol_map = parse_symbol_table(list_file)
    #print(f"Found {len(symbol_map)} symbols")

    # Read all symbols for improved matching
    try:
        with open(list_file, 'r', encoding='utf-8', errors='ignore') as f:
            list_symbols = f.read()
    except Exception as e:
        print(f"Error reading symbol file for matching: {e}")
        list_symbols = ""

    # Process romlib.def file
    #print(f"Processing input file: {input_file}")
    process_romlib_def(input_file, output_file, "", symbol_map, list_symbols)

    # Process romlib_driver.def file
    input_driver_file = os.path.join(script_dir, "romlib_driver.def")
    output_driver_file = os.path.join(script_dir, "romlib_driver.h")
    
    if os.path.exists(input_driver_file):
        #print(f"Processing input file: {input_driver_file}")
        process_romlib_def(input_driver_file, output_driver_file, None, symbol_map, list_symbols)
        print("[OK] romlib.h, romlib.inc and romlib_driver.h generation completed!")
    else:
        print("[OK] romlib.h and romlib.inc generation completed!")


if __name__ == "__main__":
    main()
