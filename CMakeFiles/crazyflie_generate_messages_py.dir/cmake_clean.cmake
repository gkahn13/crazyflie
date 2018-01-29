FILE(REMOVE_RECURSE
  "CMakeFiles/crazyflie_generate_messages_py"
  "devel/lib/python3/dist-packages/crazyflie/msg/_CFMotion.py"
  "devel/lib/python3/dist-packages/crazyflie/msg/_CFCommand.py"
  "devel/lib/python3/dist-packages/crazyflie/msg/_CFData.py"
  "devel/lib/python3/dist-packages/crazyflie/msg/__init__.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/crazyflie_generate_messages_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
