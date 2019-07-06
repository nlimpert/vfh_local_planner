## *********************************************************
##
## File autogenerated for the vfh_local_planner package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 246, 'name': 'Default', 'parent': 0, 'srcfile': '/home/nlimpert/dev/pre_ws/src/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 274, 'description': 'The minimum distance the robot is allowed to get to obstacles when stopped', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/nlimpert/dev/pre_ws/src/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'm_safety_dist_0ms', 'edit_method': '', 'default': 100.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 274, 'description': 'The minimum distance the robot is allowed to get to obstacles when stopped', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/nlimpert/dev/pre_ws/src/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'm_safety_dist_1ms', 'edit_method': '', 'default': 100.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 274, 'description': 'The maximum allowable speed of the robot', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/nlimpert/dev/pre_ws/src/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'm_max_speed', 'edit_method': '', 'default': 200.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 274, 'description': 'The maximum allowable speed of the robot through a narrow opening', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/nlimpert/dev/pre_ws/src/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'm_max_speed_narrow_opening', 'edit_method': '', 'default': 200.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 274, 'description': 'The maximum allowable speed of the robot through a wide opening', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/nlimpert/dev/pre_ws/src/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'm_max_speed_wide_opening', 'edit_method': '', 'default': 300.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 274, 'description': 'The maximum allowable acceleration of the robot.', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/nlimpert/dev/pre_ws/src/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'm_max_acceleration', 'edit_method': '', 'default': 200.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 274, 'description': 'The minimum allowable turnrate of the robot.', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/nlimpert/dev/pre_ws/src/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'm_min_turnrate', 'edit_method': '', 'default': 40.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 274, 'description': 'The maximum allowable turnrate of the robot when stopped.', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/nlimpert/dev/pre_ws/src/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'm_max_turnrate_0ms', 'edit_method': '', 'default': 40.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 274, 'description': 'The maximum allowable turnrate of the robot when travelling 1 m/s.', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/nlimpert/dev/pre_ws/src/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'm_max_turnrate_1ms', 'edit_method': '', 'default': 40.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 274, 'description': 'The higher the value, the closer the robot will get to obstacles before avoiding (while stopped).', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/nlimpert/dev/pre_ws/src/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'm_free_space_cutoff_0ms', 'edit_method': '', 'default': 2000000.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 274, 'description': 'The higher the value, the closer the robot will get to obstacles before avoiding (when travelling 1 m/s.).', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/nlimpert/dev/pre_ws/src/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'm_free_space_cutoff_1ms', 'edit_method': '', 'default': 2000000.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 274, 'description': 'histogram threshold', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/nlimpert/dev/pre_ws/src/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'm_obs_cutoff_0ms', 'edit_method': '', 'default': 4000000.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 274, 'description': 'histogram threshold', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/nlimpert/dev/pre_ws/src/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'm_obs_cutoff_1ms', 'edit_method': '', 'default': 4000000.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 274, 'description': 'Bias for the robot to turn to move toward goal position.', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/nlimpert/dev/pre_ws/src/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'm_weight_desired_dir', 'edit_method': '', 'default': 5.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 274, 'description': 'Bias for the robot to continue moving in current direction of travel', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/nlimpert/dev/pre_ws/src/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'm_weight_current_dir', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 274, 'description': 'robot radius, 300', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/nlimpert/dev/pre_ws/src/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'm_robot_radius', 'edit_method': '', 'default': 0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 274, 'description': 'Retore to the default configuration', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/nlimpert/dev/pre_ws/src/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'restore_defaults', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

