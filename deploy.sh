#!/bin/bash

verbosity=0
version=0.0.0
owner="trinamic"
repo="TMC-EvalSystem"
api="https://api.github.com/repos/${owner}/${repo}"
token="changeme"
notify="git_Eval_News@trinamic.com"
smtp_server="kug.is"
smtp_user="tmc@kug.is"
smtp_pass="changeme"
flags_set=false

usage() {
  echo "Github release deploy script"
  echo "Usage:"
  echo -e "\t ./deploy.sh [flags]"
  echo "Flags:"
  echo -e "\t-h"
  echo -e "\t\tPrint this message."
  echo -e "\t-v"
  echo -e "\t\tIncrease verbosity level by one per 'v'."
  echo -e "\t-t token"
  echo -e "\t\tGithub APIv3 token."
  echo -e "\t-n notify_email"
  echo -e "\t\t Add notify_email to the list of emails to be notified about this new version. This requires mutt to be installed."
  echo -e "\t-s smtp_server"
  echo -e "\t\t SMTP server used to send notifications."
  echo -e "\t-u smtp_user"
  echo -e "\t\t SMTP user used to send notifications."
  echo -e "\t-w smtp_pass"
  echo -e "\t\t SMTP password used to send notifications."
  echo "Examples:"
  echo -e "\t./deploy.sh -vv -t <secret>"
}

v_echo() {
  if [ $verbosity -ge $2 ]; then
    echo -e "${1}"
  fi
}

while getopts 'vt:s:u:w:n:h' flag; do
  case "${flag}" in
    v)
      ((verbosity++));
      ;;
    t)
      token="${OPTARG}"
      ;;
    s)
      smtp_server="${OPTARG}"
      ;;
    u)
      smtp_user="${OPTARG}"
      ;;
    w)
      smtp_pass="${OPTARG}"
      ;;
    n)
      notify="${notify},${OPTARG}"
      ;;
    *)
      usage
      exit 1
      ;;
  esac
  flags_set=true
done
shift $((OPTIND -1))

if ! $flags_set; then
  usage
  exit 1
fi

v_echo "verbosity: ${verbosity}" 2
v_echo "owner: ${owner}" 2
v_echo "repo: ${repo}" 2
v_echo "api: ${api}" 2
v_echo "token: ${token}" 2
v_echo "notify: ${notify}" 2
v_echo "smtp_server: ${smtp_server}" 2
v_echo "smtp_user: ${smtp_user}" 2
v_echo "smtp_pass: ${smtp_pass}" 2

v_echo "Reading current version from version.txt..." 2
version=$(cat version.txt)
version=${version#*=}
v_echo "Current version: ${version}" 1

v_echo "Configuring git..." 1
v_echo "Creating git tag..." 2
git tag $version
v_echo "Pushing tags..." 2
git push --tags

v_echo "Validating token..." 1
curl -o /dev/null -sH "Authorization: token ${token}" "${api}" || { echo "Error: Invalid repo, token or network issue!";  exit 1; }

v_echo "Uploading assets..." 1
curl -X POST --data "{\"tag_name\":\"${version}\"}" -H "Authorization: token ${token}" "${api}/releases"
# dirty hack:
release_id=$(curl -sH "Authorization: token ${token}" "${api}/releases/tags/${version}" | tr '\n' '\r' | sed -n "s/\([^,\n]*\/releases\/\)\([0-9]*\)\(.*\)/\2/p" | tr '\n' '\r')

# Uploading to GitHub

asset="https://uploads.github.com/repos/${owner}/${repo}/releases/${release_id}/assets?name="

v_echo "Release id: ${release_id}" 2
v_echo "Uploading Landungsbruecke.hex..." 2
curl --data-binary @"_build_Landungsbruecke/Landungsbruecke_v${version}_BL.hex" -H "Authorization: token ${token}" -H "Content-Type: application/octet-stream" "${asset}Landungsbruecke.hex"
v_echo "Uploading Startrampe.hex..." 2
curl --data-binary @"_build_Startrampe/Startrampe_v${version}_BL.hex" -H "Authorization: token ${token}" -H "Content-Type: application/octet-stream" "${asset}Startrampe.hex"

# Notifying people
if [ -n $notify ]; then
  github_release="https://github.com/${owner}/${repo}/releases/latest"
  v_echo "Sending notification emails to ${notify}..." 1
  mailsend-go -smtp $smtp_server -port 587 auth -user $smtp_user -pass $smtp_pass -f $smtp_user -t "leonardkugis@gmail.com" -cc $notify -sub "New ${repo} version deployed" body -msg "<html><a href=\"${github_release}\">${version}</a></html>" -debug
fi

v_echo "Increasing current version level 3 by 1..." 2
version="${version%.*}.$((${version##*.}+1))"
v_echo "New version: ${version}" 1
v_echo "Storing new version in version.txt..." 2
echo "VERSION=${version}" > version.txt
