# Contributing to MW-Lib

Thank you for your interest in contributing to MW-Lib! This document provides guidelines for contributing to this FRC library.

## Development Environment

### Prerequisites
- Java 17 or later
- Git

### Setup
```bash
git clone https://github.com/FRC-Team-4143/MW-Lib.git
cd MW-Lib
./gradlew build
```

## Code Style

- Follow existing Java conventions
- Use descriptive variable and method names
- Add JavaDoc comments for public APIs
- Keep line length under 100 characters when reasonable

## Testing

- Run tests before submitting: `./gradlew test`
- Test with actual FRC hardware when possible
- Add unit tests for new functionality

## Submitting Changes

1. **Fork the repository**
2. **Create a feature branch**: `git checkout -b feature/your-feature-name`
3. **Make your changes**
4. **Test thoroughly**: `./gradlew build test`
5. **Commit with clear messages**
6. **Push to your fork**
7. **Open a Pull Request**

## Issue Guidelines

### Bug Reports
- Use a clear, descriptive title
- Include steps to reproduce
- Provide code examples
- Mention your environment (Java version, FRC tools version, etc.)

### Feature Requests
- Explain the use case
- Describe the proposed solution
- Consider alternative solutions

## Questions?

Feel free to open an issue for questions or join the FRC community discussions!

## License

By contributing, you agree that your contributions will be licensed under the MIT License.