#!/bin/bash

if [ -z "$(which checklink)" ]; then
    echo "ERROR: checklink not found in PATH, install w3c-linkchecker for HTML link validation" 1>&2
    echo "Can be downloaded from https://github.com/w3c/link-checker." 1>&2
    echo "link-checker/bin/checklink is a perlscript, nothing else is needed." 1>&2
    exit 1
fi

# Do the rest from the docs directory
cd "$(dirname "$0")/../docs" || { echo "Could not change directory to '$(dirname "$0")/../docs'"; exit 1; }

CHKOPT=(--exclude "(http|https|irc)://" --quiet --follow-file-links)

#Note: grep is used to filer out an error message due to a bug in checklink in debian/ubuntu
#should be removed as soon as this package is fixed (upstream is already fine)

retval=0
if [ $# -gt 0 ]; then
    # Only process individual files if passed on the command line.
    for f in "$@"; do
        if [ -r "$f" ]; then
            checklink "${CHKOPT[@]}" "$f" 2>&1 | grep -vE 'Use of uninitialized value .* at .*checklink line [0-9]+'
            ret="${PIPESTATUS[0]}"
            if [ "$ret" -ne 0 ]; then
                echo "'$f': File check: Fail"
                retval="$ret"
            else
                echo "'$f': File check: OK"
            fi
        else
            echo "Cannot read file '$f'"
            retval=1
        fi
    done
else
    #Otherwhise, recursively check build/html/index.html
    f="build/html/index.html"
    if [ -r "$f" ]; then
        checklink --recursive "${CHKOPT[@]}" "$f" 2>&1 | grep -vE 'Use of uninitialized value .* at .*checklink line [0-9]+'
        ret="${PIPESTATUS[0]}"
        if [ "$ret" -ne 0 ]; then
            echo "'$f': Recursive check: Fail"
            retval="$ret"
        else
            echo "'$f': Recursive check: OK"
        fi
    else
        echo "Cannot read file '$f', did you build the doc first?"
        retval=1
    fi
fi

exit "$retval"
