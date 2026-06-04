#!/bin/sh
# Test double for pixelpilot. Appends its argv to $PP_ARGV_FILE (if set), then
# either exits non-zero (when --die is present) or sleeps to stay "running".
[ -n "$PP_ARGV_FILE" ] && echo "$@" >> "$PP_ARGV_FILE"
case " $* " in
  *" --die "*) exit 7 ;;
esac
exec sleep 30
