### Parameters ###
Hz = 30
LinearKp = 0.73
AngularKp = 1.3

MinXLinearSpeed = 0.0       # [m/s]    후진기능을 넣을 시 음수로 전환
MaxXLinearSpeed = 0.3       # [m/s]
MaxYLinearSpeed = 0.3       # [m/s]
MaxAngularSpeed = 30        # [deg/s]

ScanRotateSpeed = 12        # [deg/s]

ThetaErrorBoundary = 1     # 회전할 때 각도 명령 허용 오차

angularAcc = 2.1        #[deg/s^2]
linearXAcc = 0.06       #[m/s^2]
linearYAcc = 0.3        #[m/s^2]
####################
dt = 1 / Hz