# Publishing MW-Lib to GitHub Packages

## Prerequisites

1. **GitHub Personal Access Token (PAT)** with the following permissions:
   - `write:packages`
   - `read:packages`
   
   Create one at: https://github.com/settings/tokens

2. **Java 17** (already configured in your project)

3. **Gradle** (using the gradlew wrapper in your project)

## Setup (One-time)

Create a `.publish-config` file in the project root with your credentials:

```bash
# Copy the template
cp .publish-config.template .publish-config

# Edit and add your credentials
nano .publish-config  # or use your preferred editor
```

Your `.publish-config` should look like:

```bash
GITHUB_ACTOR=your_github_username
GITHUB_TOKEN=ghp_your_github_token_here
DEFAULT_VERSION=1.0.0  # optional
```

**Note:** `.publish-config` is in `.gitignore` and will not be committed to the repository.

## Publishing

The unified `publish.sh` script supports multiple modes:

### Interactive Mode (Default)

```bash
./publish.sh
```

Prompts for any missing credentials and version, runs all steps with full output.

### Quick Mode with Version

```bash
./publish.sh 1.0.0
```

Uses credentials from `.publish-config`, publishes version 1.0.0 with full output.

### Quiet Mode

```bash
./publish.sh -q 1.0.0
```

Minimal output, no prompts, uses config file or environment variables.

### Skip Tests

```bash
./publish.sh --skip-tests 1.0.0
```

Skips running tests to speed up publishing.

### All Options

```bash
./publish.sh -q --skip-tests 1.0.0
```

Quiet mode with tests skipped.

### Help

```bash
./publish.sh --help
```

Shows all available options and usage examples.

## Direct Gradle Command

If you prefer to run Gradle directly, load credentials first:

```bash
# Load from config file
source .publish-config

# Optional: Update version in publish.gradle first
# Then run:
./gradlew clean build publish
```

Or set environment variables:

```bash
export GITHUB_TOKEN="your_token"
export GITHUB_ACTOR="your_username"
./gradlew clean build publish
```

## Verifying Publication

After publishing, your package will be available at:
```
https://github.com/FRC-Team-4143/MW-Lib/packages
```

## Using the Published Package

To use this library in another project, add to your `build.gradle`:

```gradle
repositories {
    maven {
        url = uri('https://maven.pkg.github.com/FRC-Team-4143/MW-Lib')
        credentials {
            username = project.findProperty('gpr.user') ?: System.getenv('GITHUB_ACTOR')
            password = project.findProperty('gpr.key') ?: System.getenv('GITHUB_TOKEN')
        }
    }
}

dependencies {
    implementation 'io.github.frc-team-4143:MW-Lib-java:1.0.0'
}
```

## Troubleshooting

### Authentication Failed
- Ensure your GitHub token has `write:packages` permission
- Check that your token hasn't expired
- Verify your username is correct

### Version Already Exists
- GitHub Packages doesn't allow overwriting existing versions
- Use a new version number or delete the old package version first

### Build Failures
- Check test results: `./gradlew test`
- Review build logs for specific errors
- Ensure all dependencies are available

## Environment Variables

You can also set credentials in `~/.gradle/gradle.properties`:

```properties
gpr.user=your_github_username
gpr.key=your_github_token
```

This way you don't need to export environment variables each time.
