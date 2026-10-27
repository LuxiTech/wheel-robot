# Source this file after defining ROOT. Do not execute it directly.
for parent in "$ROOT/packages" "$ROOT/ros2_ws/src" "$ROOT/apps"; do
  for folder in "$parent"/*; do
    name=$(basename "$folder")
    if [[ -f "$folder/$name/__init__.py" ]]; then
      export PYTHONPATH="$folder${PYTHONPATH:+:$PYTHONPATH}"
    fi
  done
done
