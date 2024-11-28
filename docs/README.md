# AVR TChan Windsurf Documentation

## Directory Structure

```
docs/
├── app/                    # Application-level documentation
├── config/                 # Configuration and build system docs
├── core/                   # Core system documentation
│   ├── event/             # Event system documentation
│   └── module/            # Module system documentation
├── drivers/               # Driver documentation
│   ├── mcu/              # MCU-specific driver docs
│   └── motor/            # Motor driver docs
├── platform/              # Platform-specific documentation
└── utils/                 # Utilities documentation
    ├── common/           # Common utilities docs
    └── debug/            # Debug utilities docs

## Documentation Guidelines

1. **File Organization**
   - Each component should have its own README.md
   - Implementation details should be in separate files
   - Examples should be in dedicated files

2. **Documentation Standards**
   - Use markdown for all documentation
   - Include code examples where applicable
   - Keep documentation up to date with code changes

3. **Cross-referencing**
   - Use relative links to reference other docs
   - Reference specific code files when needed
   - Maintain a clear hierarchy of documentation

## Quick Links

- [Error Code Guidelines](utils/common/error_code_guidelines.md)
- [HAL Architecture](platform/hal_architecture.md)
- [Driver Organization Patterns](drivers/driver_organization_patterns.md)
