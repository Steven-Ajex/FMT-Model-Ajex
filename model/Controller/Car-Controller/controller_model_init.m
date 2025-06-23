model_version = 'v0.0.1';
model_name = 'Car Controller';

%% load configuration
load('control_default_config.mat');

%% Constant Variable
CONTROL_CONST.dt = 0.01;   % model execution period

%% Exported Value
CONTROL_EXPORT_VALUE.period = uint32(CONTROL_CONST.dt*1e3);
CONTROL_EXPORT_VALUE.model_info = int8([model_name, ' ', model_version, 0]); % 0 for end of string
% Export to firmware
CONTROL_EXPORT = Simulink.Parameter(CONTROL_EXPORT_VALUE);
CONTROL_EXPORT.CoderInfo.StorageClass = 'ExportedGlobal';

% Airframe id
AIRFRAME = 2;

%% Paramaters
CONTROL_PARAM_VALUE.VEL_P = single(2);
CONTROL_PARAM_VALUE.VEL_I = single(2);
CONTROL_PARAM_VALUE.VEL_D = single(0);
CONTROL_PARAM_VALUE.VEL_I_MAX = single(0.1);
CONTROL_PARAM_VALUE.VEL_I_MIN = single(-0.1);
CONTROL_PARAM_VALUE.VEL_D_MAX = single(0.1);
CONTROL_PARAM_VALUE.VEL_D_MIN = single(-0.1);
CONTROL_PARAM_VALUE.R_P = single(5);
CONTROL_PARAM_VALUE.R_I = single(5);
CONTROL_PARAM_VALUE.R_D = single(0);
CONTROL_PARAM_VALUE.R_I_MAX = single(0.1);
CONTROL_PARAM_VALUE.R_I_MIN = single(-0.1);
CONTROL_PARAM_VALUE.R_D_MAX = single(0.1);
CONTROL_PARAM_VALUE.R_D_MIN = single(-0.1);

CONTROL_PARAM_VALUE.THROTTLE_SCALE = single(300);
CONTROL_PARAM_VALUE.THROTTLE_BIAS = single(1500);
CONTROL_PARAM_VALUE.THROTTLE_BIAS1 = single(1500);
CONTROL_PARAM_VALUE.SERVO_SCALE = single(200);
CONTROL_PARAM_VALUE.SERVO_BIAS = single(1500);
CONTROL_PARAM_VALUE.SERVO_MAX_DIFF_UP = single(200);
CONTROL_PARAM_VALUE.SERVO_MAX_DIFF_LOW = single(200);
CONTROL_PARAM_VALUE.DR_SCALE = single(400);
CONTROL_PARAM_VALUE.DR_FW_START = single(1600);
CONTROL_PARAM_VALUE.DR_BW_START = single(1400);

% Export to firmware
CONTROL_PARAM = Simulink.Parameter(CONTROL_PARAM_VALUE);
CONTROL_PARAM.CoderInfo.StorageClass = 'ExportedGlobal';