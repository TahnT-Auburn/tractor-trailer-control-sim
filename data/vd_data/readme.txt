Double Lane Changes:
VehicleDynamics_31_12_2002_____18_19_09
VehicleDynamics_31_12_2002_____18_24_00
VehicleDynamics_31_12_2002_____20_01_15
VehicleDynamics_31_12_2002_____20_09_23    
VehicleDynamics_31_12_2002_____20_14_02
VehicleDynamics_31_12_2002_____20_16_35
VehicleDynamics_31_12_2002_____20_22_54    
VehicleDynamics_31_12_2002_____21_21_04
VehicleDynamics_31_12_2002_____21_22_24
VehicleDynamics_31_12_2002_____21_41_05
VehicleDynamics_31_12_2002_____21_50_19

Appear to be Understeer Tests:
VehicleDynamics_31_12_2002_____21_24_02
VehicleDynamics_31_12_2002_____21_42_47

Other Tests:
VehicleDynamics_31_12_2002_____18_32_01
VehicleDynamics_31_12_2002_____18_35_32 

Not includeded in File (but appear to have good data):
VehicleDynamics_31_12_2002_____18_33_21
VehicleDynamics_31_12_2002_____18_33_48
VehicleDynamics_31_12_2002_____18_34_27
VehicleDynamics_31_12_2002_____18_34_57
VehicleDynamics_31_12_2002_____18_35_32   
VehicleDynamics_31_12_2002_____18_36_22
VehicleDynamics_31_12_2002_____18_37_10


To Fix the Yaw Rate:

for k = 1:length(val)
	if val(k) > 50
        	val(k) = val(k) - (2^8 - 1)*0.54;
        end
end

