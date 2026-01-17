#!/bin/bash

# Script to publish MW-Lib to GitHub Packages Maven Repository
# This script should be run from the root of the MW-Lib project

set -e  # Exit on error

echo "======================================"
echo "MW-Lib GitHub Packages Publisher"
echo "======================================"
echo

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check for required tools
if ! command_exists gradle && ! command_exists ./gradlew; then
    echo -e "${RED}Error: Neither gradle nor gradlew found!${NC}"
    exit 1
fi

# Determine which gradle to use
if [ -x "./gradlew" ]; then
    GRADLE_CMD="./gradlew"
    echo -e "${GREEN}Using local gradlew wrapper${NC}"
else
    GRADLE_CMD="gradle"
    echo -e "${GREEN}Using system gradle${NC}"
fi

# Load credentials from config file if it exists
CONFIG_FILE=".publish-config"
if [ -f "$CONFIG_FILE" ]; then
    echo -e "${GREEN}Loading credentials from $CONFIG_FILE${NC}"
    source "$CONFIG_FILE"
fi

# Check for GitHub token
if [ -z "$GITHUB_TOKEN" ]; then
    echo -e "${YELLOW}GITHUB_TOKEN not found in config file or environment.${NC}"
    echo
    echo "Please provide your GitHub Personal Access Token (PAT)."
    echo "The token needs 'write:packages' and 'read:packages' permissions."
    echo
    echo "To create a token:"
    echo "1. Go to https://github.com/settings/tokens"
    echo "2. Click 'Generate new token' > 'Generate new token (classic)'"
    echo "3. Select scopes: 'write:packages' and 'read:packages'"
    echo "4. Generate and copy the token"
    echo
    echo -e "${YELLOW}Tip: Save credentials to $CONFIG_FILE to avoid entering them each time${NC}"
    echo
    read -sp "Enter your GitHub token: " GITHUB_TOKEN
    echo
    
    if [ -z "$GITHUB_TOKEN" ]; then
        echo -e "${RED}Error: GitHub token is required!${NC}"
        exit 1
    fi
fi

export GITHUB_TOKEN

# Get GitHub username
if [ -z "$GITHUB_ACTOR" ]; then
    echo
    read -p "Enter your GitHub username: " GITHUB_ACTOR
    
    if [ -z "$GITHUB_ACTOR" ]; then
        echo -e "${RED}Error: GitHub username is required!${NC}"
        exit 1
    fi
fi

export GITHUB_ACTOR

if [ "$QUIET" = false ]; then
    echo
    echo -e "${GREEN}Configuration:${NC}"
    echo "  GitHub User: $GITHUB_ACTOR"
    echo "  Token: ${GITHUB_TOKEN:0:4}****"
    echo
fi

# Ask for version if not set
if [ -z "$VERSION" ]; then
    DEFAULT_MSG="${DEFAULT_VERSION:-1.0.0-dev}"
    
    if [ "$INTERACTIVE" = true ]; then
        echo "Current version in publish.gradle will be used (default: $DEFAULT_MSG)"
        read -p "Enter version to publish (or press Enter to use default): " VERSION
    fi
    
    # Use DEFAULT_VERSION from config if VERSION is still empty
    if [ -z "$VERSION" ]; then
        VERSION="$DEFAULT_MSG"
    fi
fi

[ "$QUIET" = false ] && echo -e "${YELLOW}Publishing version: $VERSION${NC}"

# Update version in publish.gradle
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS
    sed -i '' "s/def pubVersion = '.*'/def pubVersion = '$VERSION'/" publish.gradle
else
    # Linux
    sed -i "s/def pubVersion = '.*'/def pubVersion = '$VERSION'/" publish.gradle
fi

echo
[ "$QUIET" = false ] && echo -e "${GREEN}Step 1: Cleaning previous builds...${NC}"
$GRADLE_CMD clean $([ "$QUIET" = true ] && echo "-q")

if [ "$SKIP_TESTS" = false ]; then
    echo
    [ "$QUIET" = false ] && echo -e "${GREEN}Step 2: Running tests...${NC}"
    $GRADLE_CMD test --continue $([ "$QUIET" = true ] && echo "-q") || {
        echo -e "${YELLOW}Warning: Some tests failed, but continuing...${NC}"
    }
else
    [ "$QUIET" = false ] && echo -e "${BLUE}Skipping tests...${NC}"
fi

echo
[ "$QUIET" = false ] && echo -e "${GREEN}Step $([ "$SKIP_TESTS" = false ] && echo "3" || echo "2"): Building library...${NC}"
$GRADLE_CMD build $([ "$QUIET" = true ] && echo "-q")

echo
[ "$QUIET" = false ] && echo -e "${GREEN}Step $([ "$SKIP_TESTS" = false ] && echo "4" || echo "3"): Publishing to GitHub Packages...${NC}"
$GRADLE_CMD publish $([ "$QUIET" = true ] && echo "-q" || echo "--info")

if [ "$QUIET" = false ]; then
    echo
    echo -e "${GREEN}======================================"
    echo "✓ Successfully published to GitHub Packages!"
    echo "======================================${NC}"
    echo
    echo "Your package should now be available at:"
    echo "https://github.com/FRC-Team-4143/MW-Lib/packages"
    echo
    echo "To use this package in another project, add this to your build.gradle:"
    echo
    echo "repositories {"
    echo "    maven {"
    echo "        url = uri('https://maven.pkg.github.com/FRC-Team-4143/MW-Lib')"
    echo "        credentials {"
    echo "            username = project.findProperty('gpr.user') ?: System.getenv('GITHUB_ACTOR')"
    echo "            password = project.findProperty('gpr.key') ?: System.getenv('GITHUB_TOKEN')"
    echo "        }"
    echo "    }"
    echo "}"
    echo
    echo "dependencies {"
    echo "    implementation 'io.github.frc-team-4143:MW-Lib-java:$VERSION'"
    echo "}"
else
    echo -e "${GREEN}✓ Successfully published version $VERSION${NC}"
fi
