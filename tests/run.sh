#!/bin/sh
# Scan the fake frontend from tests/fakedvb.c on Linux: make check
set -eu

cd "$(dirname "$0")/.."
make -s dvbscan
cc -std=gnu99 -Wall -Wextra -O2 -shared -fPIC -o tests/fakedvb.so tests/fakedvb.c -ldl

out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
fail=0

run() {
  fam=$1
  shift
  if ! FAKE_DVB=$fam LD_PRELOAD=$PWD/tests/fakedvb.so ./dvbscan --wait 50 --psi-ms 600 "$@" \
      >"$out/stdout" 2>"$out/stderr"; then
    echo "FAIL: dvbscan $*"
    cat "$out/stderr"
    fail=1
  fi
}

expect() {
  hay=$(tr '\n' '|' <"$out/stdout")
  needle=$(printf '%s' "$1" | tr '\n' '|')
  case $hay in
  *"$needle"*) ;;
  *)
    echo "FAIL: no '$1' in output of $fam scan"
    cat "$out/stdout"
    fail=1
    ;;
  esac
}

run c -o "$out/c.log"
expect "system=dvbca freq=306 sr=6875 mod=qam256"
expect "system=dvbca freq=314 sr=6900 mod=qam64"
expect "program 1     Cable One  [Fake]"
expect "  system dvbca;
  frequency 306000000;
  symbol_rate 6875;
  modulation qam256;"

run t -o "$out/t.log"
expect "system=dvbt freq=602 bw=8"
expect "system=dvbt2 freq=650 bw=8"
expect "program 1     Terr Two HD  [Fake]"
expect "dvb_card t0_650 {"
expect "  system dvbt2;
  frequency 650000000;
  bandwidth 8000000;"

run s -s s --from 11700 --to 12300 --sr 27500 -o "$out/s.log"
expect "system=dvbs2 freq=11766 pol=h sr=27500 mod=psk8 rolloff=20"
expect "system=dvbs freq=12226 pol=v sr=27500 mod=qpsk"
expect "program 1     Sat One HD  [Fake]"
expect "dvb_card s0_11766h {"
expect "  frequency 11766;
  symbol_rate 27500;
  polarization h;
  modulation psk8;
  rolloff 20;"
expect "# programs 2, with names 2"

run s --list "$out/s.log" -o "$out/s2.log"
expect "system=dvbs2 freq=11766 pol=h sr=27500 mod=psk8"
expect "# programs 2, with names 2"

printf 'system=dvbt2 freq=650 bw=8 plp=0\n' >"$out/t.list"
run t --list "$out/t.list" -o "$out/t2.log"
expect "  plp_stream_id 0;"

[ $fail = 0 ] && echo "all scans ok"
exit $fail
