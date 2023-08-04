#!/usr/bin/env bash

usage() {
	cat <<EOF
usage: ${0} CANEDUDEV_URL

Inserts CANEDUDEV_URL where %DOMAIN% is defined in rst files.
CANEDUDEV_URL can be set as environment variable as well.
EOF
	exit 0
}

if [[ ${1} == '-h' || ${1} == '--help' ]]; then
	usage

elif [[ -n ${1} ]]; then
	REPLACE_VAR="$1"

elif [[ -n ${CANEDUDEV_URL} ]]; then
	REPLACE_VAR=${CANEDUDEV_URL}

else
	usage

fi

FIND_VAR="%DOMAIN%"

find build/docs/html -name "*.html" -exec sed -i "s/${FIND_VAR}/${REPLACE_VAR}/g" {} \;
