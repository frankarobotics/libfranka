#!/usr/bin/env bash
# grep the result of clang-format with -output-replacements-xml flag
# if any "replacement" tags are found, then code is incorrectly formatted
CLANG_FORMAT=$1
shift
# skip the -output-replacements parameter
shift
FILES="$@"

# Array to hold files with formatting issues
FILES_WITH_ISSUES=()

# Check each file
for file in $FILES; do
  # Run clang-format and check if replacements are needed
  OUTPUT=$($CLANG_FORMAT -output-replacements-xml "$file")

  # If there are replacements, add the file to the issues array
  if [[ $OUTPUT == *"<replacement "* ]]; then
    FILES_WITH_ISSUES+=("$file")
  fi
done

# If any files had issues, print them and exit with an error
if [ ${#FILES_WITH_ISSUES[@]} -ne 0 ]; then
  echo "Please reformat the following files clang-format:"
  for file in "${FILES_WITH_ISSUES[@]}"; do
    echo "  - $file"
  done
  exit 1
fi
