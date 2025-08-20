# .gitattributes

# Exclude all files by default from language statistics
* linguist-detectable=false

# Only include files in the src directory
src/** linguist-detectable=true

# Optional: If you want to exclude specific file types even in src/
# src/**/*.md linguist-documentation
# src/**/*.txt linguist-documentation

# Optional: If you have documentation files you want to mark as such
*.md linguist-documentation
*.html linguist-documentation
*.mermaid linguist-documentation
docs/** linguist-documentation
examples/** linguist-documentation
README* linguist-documentation