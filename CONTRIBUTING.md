# Contributing to Windsurf

Thank you for your interest in contributing to the RecycleCtrl project. This document provides guidelines for code organization and contribution standards.

## Code Organization

The codebase is organized into the following main directories:

- `src/app/`: Application-level code and initialization
- `src/config/`: Configuration files and system settings
- `src/core/`: Core system functionality
- `src/drivers/`: Hardware-specific drivers
- `src/modules/`: Feature-specific modules
- `src/platform/`: Platform-specific implementations
- `src/test/`: Test files and test utilities
- `src/utils/`: Common utilities and helper functions

### Directory Structure Guidelines

1. **New Features**
   - Place new features in the appropriate existing directory based on functionality
   - Create new subdirectories in `modules/` for substantial new features
   - Keep related files together in the same directory

2. **File Naming**
   - Use lowercase with underscores for file names: `feature_name.c`
   - Use prefixes to indicate the type of file:
     - `drv_`: Driver implementations
     - `cfg_`: Configuration files
     - `ctrl_`: Control modules
     - `test_`: Test files
     - `plat_`: Platform-specific code

## Code Style Guidelines

### File Organization

1. **Header Files**
   - Include guards using `#ifndef`, `#define`, `#endif`
   - Group includes in the following order:
     1. Standard library headers
     2. AVR specific headers
     3. Project configuration headers
     4. Platform headers
     5. Driver headers
     6. Module headers
   - Separate groups with a blank line

2. **Source Files**
   - Start with file description comment block
   - Follow with includes
   - Define constants and types
   - Declare static variables
   - Implement functions

### Function Organization

1. **Function Grouping**
   - Group related functions together
   - Order functions from high-level to low-level
   - Place helper functions near the main functions that use them

2. **Function Structure**
   - Begin with input validation
   - Include error handling
   - Document return values and error codes

### Documentation

1. **Comments**
   - Add a descriptive header to each file
   - Document all functions with:
     - Purpose
     - Parameters
     - Return values
     - Error conditions
   - Use inline comments for complex logic

2. **Error Handling**
   - Use the defined error codes from `error_codes.h`
   - Document all possible error conditions
   - Implement proper error recovery where applicable

## Testing

1. **Test Files**
   - Place test files in the `test/` directory
   - Name test files with `test_` prefix
   - Create corresponding test utilities as needed

## Safety and Security

1. **Buffer Safety**
   - Use secure buffer functions for data handling
   - Implement proper bounds checking
   - Validate all input data

2. **Hardware Access**
   - Use appropriate driver abstractions
   - Implement proper initialization sequences
   - Handle hardware failures gracefully

## Contribution Process

1. **Before Contributing**
   - Review existing code in the target area
   - Ensure new code follows established patterns
   - Test thoroughly

2. **Code Review**
   - Ensure code meets all guidelines
   - Document all changes
   - Include test cases for new functionality

## Version Control

1. **Commits**
   - Use clear, descriptive commit messages
   - Keep commits focused and atomic
   - Reference related issues or tickets

2. **Branches**
   - Create feature branches for new development
   - Use descriptive branch names
   - Keep branches up to date with main

## Questions and Support

If you have questions about these guidelines or need clarification, please open an issue in the repository.
