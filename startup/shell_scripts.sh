  until rostopic list > /dev/null 2>&1; do
    echo "waiting for ros"
    sleep 1;
  done