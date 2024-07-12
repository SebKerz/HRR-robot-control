if [ -n "${KB_SCRIPT_PATH}" ]; then
{
  source "${KB_SCRIPT_PATH}/access_tokens" > /dev/null
} || { 
  echo "could not get access tokens" 
}
fi
