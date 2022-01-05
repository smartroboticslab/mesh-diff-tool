#!/bin/sh
set -eu

usage() {
	printf 'Usage: %s MESH_DIR GT_MESH_DIR\n' "${0##*/}"
}

script_dir() {
	dirname "$0"
}

mesh_compare_program() {
	printf '%s/build/compareMultiple\n' "$(script_dir)"
}

desired_scale() {
	awk '
		/^format binary/ { print 0; exit }
		/^element face/ { num_faces = $3 }
		/^element/ && $2 != "face" { line_offset += $3 }
		/^end_header$/ { in_body = 1; line_offset += NR + 1 }
		in_body && NR >= line_offset {
			// TODO compute the scale property offset instead of assuming 2
			scale = $($1 + 2)
			if (scale == 0) {
				num_desired_scale++
			}
		}
		END { print 100 * num_desired_scale / num_faces }' "$1"
}

show_desired_scale() {
	while IFS= read -r line
	do
		case "$line" in
			'Detected object'*)
				printf '%s\tDesired scale (%%)\n' "$line"
				;;
			*)
				mesh=$(printf '%s\n' "$line" | cut -f 1)
				printf '%s\t%s\n' "$line" "$(desired_scale "$mesh")"
				;;
		esac
	done
}



if [ "$#" -ne 2 ]
then
	usage
	exit 2
fi

heatmap_dir="${1%%/}/heatmaps/"
mkdir -p "$heatmap_dir"
$(mesh_compare_program) "$1" "$2" "$heatmap_dir" 2> "$1"/error.log \
	| show_desired_scale | tee "$1"/results.tsv

