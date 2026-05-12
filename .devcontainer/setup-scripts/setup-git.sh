#!/bin/bash

# Configure git with user name and email from environment variables
if [ -n "$GIT_USER_NAME" ]; then
  git config --global user.name "$GIT_USER_NAME"
  echo "Git user.name set to: $GIT_USER_NAME"
else
  echo "Warning: GIT_USER_NAME not set"
fi

if [ -n "$GIT_USER_EMAIL" ]; then
  git config --global user.email "$GIT_USER_EMAIL"
  echo "Git user.email set to: $GIT_USER_EMAIL"
else
  echo "Warning: GIT_USER_EMAIL not set"
fi