
"use strict";

let KortexError = require('./KortexError.js');
let ApiOptions = require('./ApiOptions.js');
let ErrorCodes = require('./ErrorCodes.js');
let SubErrorCodes = require('./SubErrorCodes.js');
let EncoderDerivativeParameters = require('./EncoderDerivativeParameters.js');
let LoopSelection = require('./LoopSelection.js');
let ActuatorConfig_ServiceVersion = require('./ActuatorConfig_ServiceVersion.js');
let ControlLoopParameters = require('./ControlLoopParameters.js');
let CustomDataSelection = require('./CustomDataSelection.js');
let SafetyIdentifierBankA = require('./SafetyIdentifierBankA.js');
let ControlLoopSelection = require('./ControlLoopSelection.js');
let VectorDriveParameters = require('./VectorDriveParameters.js');
let ActuatorConfig_SafetyLimitType = require('./ActuatorConfig_SafetyLimitType.js');
let PositionCommand = require('./PositionCommand.js');
let CoggingFeedforwardModeInformation = require('./CoggingFeedforwardModeInformation.js');
let Servoing = require('./Servoing.js');
let CoggingFeedforwardMode = require('./CoggingFeedforwardMode.js');
let TorqueOffset = require('./TorqueOffset.js');
let ActuatorConfig_ControlMode = require('./ActuatorConfig_ControlMode.js');
let StepResponse = require('./StepResponse.js');
let CommandModeInformation = require('./CommandModeInformation.js');
let TorqueCalibration = require('./TorqueCalibration.js');
let AxisOffsets = require('./AxisOffsets.js');
let ControlLoop = require('./ControlLoop.js');
let RampResponse = require('./RampResponse.js');
let AxisPosition = require('./AxisPosition.js');
let ActuatorConfig_ControlModeInformation = require('./ActuatorConfig_ControlModeInformation.js');
let FrequencyResponse = require('./FrequencyResponse.js');
let CommandMode = require('./CommandMode.js');
let CustomDataIndex = require('./CustomDataIndex.js');
let ActuatorCyclic_CustomData = require('./ActuatorCyclic_CustomData.js');
let ActuatorCyclic_Feedback = require('./ActuatorCyclic_Feedback.js');
let ActuatorCyclic_Command = require('./ActuatorCyclic_Command.js');
let StatusFlags = require('./StatusFlags.js');
let ActuatorCyclic_ServiceVersion = require('./ActuatorCyclic_ServiceVersion.js');
let CommandFlags = require('./CommandFlags.js');
let ActuatorCyclic_MessageId = require('./ActuatorCyclic_MessageId.js');
let ProtectionZone = require('./ProtectionZone.js');
let Wrench = require('./Wrench.js');
let IPv4Configuration = require('./IPv4Configuration.js');
let GpioAction = require('./GpioAction.js');
let WifiConfigurationList = require('./WifiConfigurationList.js');
let GpioCommand = require('./GpioCommand.js');
let NetworkNotification = require('./NetworkNotification.js');
let Base_ControlModeInformation = require('./Base_ControlModeInformation.js');
let BridgeResult = require('./BridgeResult.js');
let ConstrainedJointAngles = require('./ConstrainedJointAngles.js');
let MappingInfoNotification = require('./MappingInfoNotification.js');
let AngularWaypoint = require('./AngularWaypoint.js');
let JointTorques = require('./JointTorques.js');
let ArmStateInformation = require('./ArmStateInformation.js');
let SequenceInfoNotificationList = require('./SequenceInfoNotificationList.js');
let GripperRequest = require('./GripperRequest.js');
let FullUserProfile = require('./FullUserProfile.js');
let ShapeType = require('./ShapeType.js');
let WrenchLimitation = require('./WrenchLimitation.js');
let TrajectoryErrorType = require('./TrajectoryErrorType.js');
let ConfigurationNotificationEvent = require('./ConfigurationNotificationEvent.js');
let Base_RotationMatrixRow = require('./Base_RotationMatrixRow.js');
let JointNavigationDirection = require('./JointNavigationDirection.js');
let MapElement = require('./MapElement.js');
let WifiSecurityType = require('./WifiSecurityType.js');
let MapEvent_events = require('./MapEvent_events.js');
let NetworkEvent = require('./NetworkEvent.js');
let SequenceInformation = require('./SequenceInformation.js');
let BackupEvent = require('./BackupEvent.js');
let ControllerInputType = require('./ControllerInputType.js');
let ConstrainedOrientation = require('./ConstrainedOrientation.js');
let ProtectionZoneInformation = require('./ProtectionZoneInformation.js');
let SequenceTasksRange = require('./SequenceTasksRange.js');
let JointTorque = require('./JointTorque.js');
let ControllerNotificationList = require('./ControllerNotificationList.js');
let ControllerEventType = require('./ControllerEventType.js');
let FirmwareComponentVersion = require('./FirmwareComponentVersion.js');
let Point = require('./Point.js');
let GpioBehavior = require('./GpioBehavior.js');
let ProtectionZoneNotificationList = require('./ProtectionZoneNotificationList.js');
let ControllerNotification_state = require('./ControllerNotification_state.js');
let ActionType = require('./ActionType.js');
let LimitationType = require('./LimitationType.js');
let BridgeStatus = require('./BridgeStatus.js');
let EventIdSequenceInfoNotification = require('./EventIdSequenceInfoNotification.js');
let WifiEnableState = require('./WifiEnableState.js');
let UserProfileList = require('./UserProfileList.js');
let SnapshotType = require('./SnapshotType.js');
let GripperMode = require('./GripperMode.js');
let SequenceTasksConfiguration = require('./SequenceTasksConfiguration.js');
let ServoingModeNotification = require('./ServoingModeNotification.js');
let Admittance = require('./Admittance.js');
let TrajectoryErrorElement = require('./TrajectoryErrorElement.js');
let MapGroup = require('./MapGroup.js');
let MapList = require('./MapList.js');
let ActionHandle = require('./ActionHandle.js');
let GpioConfigurationList = require('./GpioConfigurationList.js');
let EmergencyStop = require('./EmergencyStop.js');
let WristDigitalInputIdentifier = require('./WristDigitalInputIdentifier.js');
let GpioPinConfiguration = require('./GpioPinConfiguration.js');
let Ssid = require('./Ssid.js');
let PreComputedJointTrajectoryElement = require('./PreComputedJointTrajectoryElement.js');
let FactoryEvent = require('./FactoryEvent.js');
let CartesianTrajectoryConstraint = require('./CartesianTrajectoryConstraint.js');
let JointAngle = require('./JointAngle.js');
let Action = require('./Action.js');
let TransformationRow = require('./TransformationRow.js');
let ChangeJointSpeeds = require('./ChangeJointSpeeds.js');
let RFConfiguration = require('./RFConfiguration.js');
let ControllerHandle = require('./ControllerHandle.js');
let ControllerBehavior = require('./ControllerBehavior.js');
let KinematicTrajectoryConstraints = require('./KinematicTrajectoryConstraints.js');
let BridgePortConfig = require('./BridgePortConfig.js');
let ActionNotificationList = require('./ActionNotificationList.js');
let CartesianTrajectoryConstraint_type = require('./CartesianTrajectoryConstraint_type.js');
let MapGroupHandle = require('./MapGroupHandle.js');
let Timeout = require('./Timeout.js');
let Action_action_parameters = require('./Action_action_parameters.js');
let FullIPv4Configuration = require('./FullIPv4Configuration.js');
let UserEvent = require('./UserEvent.js');
let GpioPinPropertyFlags = require('./GpioPinPropertyFlags.js');
let JointsLimitationsList = require('./JointsLimitationsList.js');
let UserProfile = require('./UserProfile.js');
let JointAngles = require('./JointAngles.js');
let ActionNotification = require('./ActionNotification.js');
let Base_RotationMatrix = require('./Base_RotationMatrix.js');
let FactoryNotification = require('./FactoryNotification.js');
let CommunicationInterfaceConfiguration = require('./CommunicationInterfaceConfiguration.js');
let OperatingMode = require('./OperatingMode.js');
let GripperCommand = require('./GripperCommand.js');
let Map = require('./Map.js');
let SequenceTaskHandle = require('./SequenceTaskHandle.js');
let ControllerNotification = require('./ControllerNotification.js');
let SwitchControlMapping = require('./SwitchControlMapping.js');
let WifiConfiguration = require('./WifiConfiguration.js');
let WrenchMode = require('./WrenchMode.js');
let Base_CapSenseConfig = require('./Base_CapSenseConfig.js');
let SequenceTasks = require('./SequenceTasks.js');
let ServoingModeNotificationList = require('./ServoingModeNotificationList.js');
let Mapping = require('./Mapping.js');
let WaypointList = require('./WaypointList.js');
let SequenceTaskConfiguration = require('./SequenceTaskConfiguration.js');
let CartesianWaypoint = require('./CartesianWaypoint.js');
let WaypointValidationReport = require('./WaypointValidationReport.js');
let SequenceTasksPair = require('./SequenceTasksPair.js');
let BridgeList = require('./BridgeList.js');
let NetworkNotificationList = require('./NetworkNotificationList.js');
let ProtectionZoneHandle = require('./ProtectionZoneHandle.js');
let UserNotificationList = require('./UserNotificationList.js');
let ControllerConfigurationList = require('./ControllerConfigurationList.js');
let Base_ControlModeNotification = require('./Base_ControlModeNotification.js');
let SoundType = require('./SoundType.js');
let ControlModeNotificationList = require('./ControlModeNotificationList.js');
let SequenceTask = require('./SequenceTask.js');
let Pose = require('./Pose.js');
let ConstrainedJointAngle = require('./ConstrainedJointAngle.js');
let MapGroupList = require('./MapGroupList.js');
let IKData = require('./IKData.js');
let BridgeConfig = require('./BridgeConfig.js');
let BluetoothEnableState = require('./BluetoothEnableState.js');
let Sequence = require('./Sequence.js');
let OperatingModeNotificationList = require('./OperatingModeNotificationList.js');
let ControllerType = require('./ControllerType.js');
let TrajectoryContinuityMode = require('./TrajectoryContinuityMode.js');
let RobotEventNotificationList = require('./RobotEventNotificationList.js');
let PasswordChange = require('./PasswordChange.js');
let JointSpeed = require('./JointSpeed.js');
let ActivateMapHandle = require('./ActivateMapHandle.js');
let ChangeWrench = require('./ChangeWrench.js');
let LedState = require('./LedState.js');
let Base_JointSpeeds = require('./Base_JointSpeeds.js');
let ProtectionZoneList = require('./ProtectionZoneList.js');
let GpioEvent = require('./GpioEvent.js');
let TransformationMatrix = require('./TransformationMatrix.js');
let ConstrainedPosition = require('./ConstrainedPosition.js');
let Orientation = require('./Orientation.js');
let TrajectoryErrorIdentifier = require('./TrajectoryErrorIdentifier.js');
let MapEvent = require('./MapEvent.js');
let Snapshot = require('./Snapshot.js');
let Gen3GpioPinId = require('./Gen3GpioPinId.js');
let TrajectoryErrorReport = require('./TrajectoryErrorReport.js');
let NetworkType = require('./NetworkType.js');
let ServoingModeInformation = require('./ServoingModeInformation.js');
let TwistCommand = require('./TwistCommand.js');
let ChangeTwist = require('./ChangeTwist.js');
let SequenceInfoNotification = require('./SequenceInfoNotification.js');
let Gripper = require('./Gripper.js');
let Base_ControlMode = require('./Base_ControlMode.js');
let SafetyEvent = require('./SafetyEvent.js');
let FirmwareBundleVersions = require('./FirmwareBundleVersions.js');
let Base_Position = require('./Base_Position.js');
let WifiInformation = require('./WifiInformation.js');
let SafetyNotificationList = require('./SafetyNotificationList.js');
let ArmStateNotification = require('./ArmStateNotification.js');
let ControllerElementHandle = require('./ControllerElementHandle.js');
let Query = require('./Query.js');
let Base_Stop = require('./Base_Stop.js');
let AppendActionInformation = require('./AppendActionInformation.js');
let NavigationDirection = require('./NavigationDirection.js');
let UserList = require('./UserList.js');
let Waypoint_type_of_waypoint = require('./Waypoint_type_of_waypoint.js');
let MappingInfoNotificationList = require('./MappingInfoNotificationList.js');
let MapHandle = require('./MapHandle.js');
let CartesianLimitationList = require('./CartesianLimitationList.js');
let IPv4Information = require('./IPv4Information.js');
let ControllerEvent = require('./ControllerEvent.js');
let NetworkHandle = require('./NetworkHandle.js');
let RobotEventNotification = require('./RobotEventNotification.js');
let ControllerElementEventType = require('./ControllerElementEventType.js');
let Waypoint = require('./Waypoint.js');
let WifiEncryptionType = require('./WifiEncryptionType.js');
let BridgeType = require('./BridgeType.js');
let Base_ServiceVersion = require('./Base_ServiceVersion.js');
let SequenceHandle = require('./SequenceHandle.js');
let ProtectionZoneNotification = require('./ProtectionZoneNotification.js');
let Xbox360DigitalInputIdentifier = require('./Xbox360DigitalInputIdentifier.js');
let WrenchCommand = require('./WrenchCommand.js');
let TrajectoryInfo = require('./TrajectoryInfo.js');
let AdmittanceMode = require('./AdmittanceMode.js');
let ControllerConfigurationMode = require('./ControllerConfigurationMode.js');
let SignalQuality = require('./SignalQuality.js');
let CartesianLimitation = require('./CartesianLimitation.js');
let OperatingModeNotification = require('./OperatingModeNotification.js');
let Faults = require('./Faults.js');
let ActionEvent = require('./ActionEvent.js');
let TwistLimitation = require('./TwistLimitation.js');
let ControllerElementHandle_identifier = require('./ControllerElementHandle_identifier.js');
let Delay = require('./Delay.js');
let UserNotification = require('./UserNotification.js');
let ZoneShape = require('./ZoneShape.js');
let ConfigurationChangeNotification = require('./ConfigurationChangeNotification.js');
let JointTrajectoryConstraint = require('./JointTrajectoryConstraint.js');
let ControllerElementState = require('./ControllerElementState.js');
let ActionExecutionState = require('./ActionExecutionState.js');
let ConfigurationChangeNotificationList = require('./ConfigurationChangeNotificationList.js');
let ConfigurationChangeNotification_configuration_change = require('./ConfigurationChangeNotification_configuration_change.js');
let BridgeIdentifier = require('./BridgeIdentifier.js');
let CartesianSpeed = require('./CartesianSpeed.js');
let Base_SafetyIdentifier = require('./Base_SafetyIdentifier.js');
let ActuatorInformation = require('./ActuatorInformation.js');
let JointLimitation = require('./JointLimitation.js');
let MappingList = require('./MappingList.js');
let Xbox360AnalogInputIdentifier = require('./Xbox360AnalogInputIdentifier.js');
let JointTrajectoryConstraintType = require('./JointTrajectoryConstraintType.js');
let ControllerConfiguration = require('./ControllerConfiguration.js');
let PreComputedJointTrajectory = require('./PreComputedJointTrajectory.js');
let RequestedActionType = require('./RequestedActionType.js');
let Base_GpioConfiguration = require('./Base_GpioConfiguration.js');
let Finger = require('./Finger.js');
let ControllerState = require('./ControllerState.js');
let MappingHandle = require('./MappingHandle.js');
let ConstrainedPose = require('./ConstrainedPose.js');
let ControllerList = require('./ControllerList.js');
let SequenceList = require('./SequenceList.js');
let SystemTime = require('./SystemTime.js');
let WifiInformationList = require('./WifiInformationList.js');
let TrajectoryInfoType = require('./TrajectoryInfoType.js');
let ServoingMode = require('./ServoingMode.js');
let ProtectionZoneEvent = require('./ProtectionZoneEvent.js');
let Twist = require('./Twist.js');
let ActionList = require('./ActionList.js');
let AdvancedSequenceHandle = require('./AdvancedSequenceHandle.js');
let Base_CapSenseMode = require('./Base_CapSenseMode.js');
let OperatingModeInformation = require('./OperatingModeInformation.js');
let RobotEvent = require('./RobotEvent.js');
let ActuatorFeedback = require('./ActuatorFeedback.js');
let BaseCyclic_Command = require('./BaseCyclic_Command.js');
let BaseFeedback = require('./BaseFeedback.js');
let ActuatorCommand = require('./ActuatorCommand.js');
let ActuatorCustomData = require('./ActuatorCustomData.js');
let BaseCyclic_Feedback = require('./BaseCyclic_Feedback.js');
let BaseCyclic_ServiceVersion = require('./BaseCyclic_ServiceVersion.js');
let BaseCyclic_CustomData = require('./BaseCyclic_CustomData.js');
let NotificationHandle = require('./NotificationHandle.js');
let UARTParity = require('./UARTParity.js');
let Permission = require('./Permission.js');
let NotificationOptions = require('./NotificationOptions.js');
let UARTStopBits = require('./UARTStopBits.js');
let CountryCode = require('./CountryCode.js');
let UARTSpeed = require('./UARTSpeed.js');
let Timestamp = require('./Timestamp.js');
let DeviceHandle = require('./DeviceHandle.js');
let CartesianReferenceFrame = require('./CartesianReferenceFrame.js');
let Empty = require('./Empty.js');
let DeviceTypes = require('./DeviceTypes.js');
let SafetyNotification = require('./SafetyNotification.js');
let SafetyHandle = require('./SafetyHandle.js');
let ArmState = require('./ArmState.js');
let UserProfileHandle = require('./UserProfileHandle.js');
let UARTDeviceIdentification = require('./UARTDeviceIdentification.js');
let SafetyStatusValue = require('./SafetyStatusValue.js');
let NotificationType = require('./NotificationType.js');
let Connection = require('./Connection.js');
let UARTWordLength = require('./UARTWordLength.js');
let CountryCodeIdentifier = require('./CountryCodeIdentifier.js');
let Unit = require('./Unit.js');
let UARTConfiguration = require('./UARTConfiguration.js');
let LinearTwist = require('./LinearTwist.js');
let ControlConfig_ControlMode = require('./ControlConfig_ControlMode.js');
let JointSpeedSoftLimits = require('./JointSpeedSoftLimits.js');
let DesiredSpeeds = require('./DesiredSpeeds.js');
let ControlConfig_ServiceVersion = require('./ControlConfig_ServiceVersion.js');
let AngularTwist = require('./AngularTwist.js');
let KinematicLimits = require('./KinematicLimits.js');
let TwistAngularSoftLimit = require('./TwistAngularSoftLimit.js');
let CartesianTransform = require('./CartesianTransform.js');
let ControlConfig_JointSpeeds = require('./ControlConfig_JointSpeeds.js');
let JointAccelerationSoftLimits = require('./JointAccelerationSoftLimits.js');
let CartesianReferenceFrameInfo = require('./CartesianReferenceFrameInfo.js');
let ToolConfiguration = require('./ToolConfiguration.js');
let ControlConfigurationNotification = require('./ControlConfigurationNotification.js');
let TwistLinearSoftLimit = require('./TwistLinearSoftLimit.js');
let PayloadInformation = require('./PayloadInformation.js');
let GravityVector = require('./GravityVector.js');
let ControlConfig_Position = require('./ControlConfig_Position.js');
let KinematicLimitsList = require('./KinematicLimitsList.js');
let ControlConfig_ControlModeInformation = require('./ControlConfig_ControlModeInformation.js');
let ControlConfigurationEvent = require('./ControlConfigurationEvent.js');
let ControlConfig_ControlModeNotification = require('./ControlConfig_ControlModeNotification.js');
let PartNumber = require('./PartNumber.js');
let SerialNumber = require('./SerialNumber.js');
let CalibrationElement = require('./CalibrationElement.js');
let RebootRqst = require('./RebootRqst.js');
let DeviceConfig_CapSenseConfig = require('./DeviceConfig_CapSenseConfig.js');
let CalibrationParameter_value = require('./CalibrationParameter_value.js');
let SafetyInformationList = require('./SafetyInformationList.js');
let PowerOnSelfTestResult = require('./PowerOnSelfTestResult.js');
let CalibrationStatus = require('./CalibrationStatus.js');
let PartNumberRevision = require('./PartNumberRevision.js');
let SafetyInformation = require('./SafetyInformation.js');
let DeviceConfig_CapSenseMode = require('./DeviceConfig_CapSenseMode.js');
let DeviceType = require('./DeviceType.js');
let MACAddress = require('./MACAddress.js');
let CapSenseRegister = require('./CapSenseRegister.js');
let CalibrationParameter = require('./CalibrationParameter.js');
let SafetyConfiguration = require('./SafetyConfiguration.js');
let FirmwareVersion = require('./FirmwareVersion.js');
let BootloaderVersion = require('./BootloaderVersion.js');
let SafetyEnable = require('./SafetyEnable.js');
let DeviceConfig_SafetyLimitType = require('./DeviceConfig_SafetyLimitType.js');
let DeviceConfig_ServiceVersion = require('./DeviceConfig_ServiceVersion.js');
let IPv4Settings = require('./IPv4Settings.js');
let Calibration = require('./Calibration.js');
let RunModes = require('./RunModes.js');
let ModelNumber = require('./ModelNumber.js');
let SafetyThreshold = require('./SafetyThreshold.js');
let SafetyConfigurationList = require('./SafetyConfigurationList.js');
let CalibrationResult = require('./CalibrationResult.js');
let CalibrationItem = require('./CalibrationItem.js');
let SafetyStatus = require('./SafetyStatus.js');
let RunMode = require('./RunMode.js');
let DeviceManager_ServiceVersion = require('./DeviceManager_ServiceVersion.js');
let DeviceHandles = require('./DeviceHandles.js');
let GripperConfig_SafetyIdentifier = require('./GripperConfig_SafetyIdentifier.js');
let RobotiqGripperStatusFlags = require('./RobotiqGripperStatusFlags.js');
let GripperCyclic_Feedback = require('./GripperCyclic_Feedback.js');
let MotorFeedback = require('./MotorFeedback.js');
let GripperCyclic_MessageId = require('./GripperCyclic_MessageId.js');
let GripperCyclic_ServiceVersion = require('./GripperCyclic_ServiceVersion.js');
let MotorCommand = require('./MotorCommand.js');
let CustomDataUnit = require('./CustomDataUnit.js');
let GripperCyclic_CustomData = require('./GripperCyclic_CustomData.js');
let GripperCyclic_Command = require('./GripperCyclic_Command.js');
let InterconnectConfig_SafetyIdentifier = require('./InterconnectConfig_SafetyIdentifier.js');
let I2CMode = require('./I2CMode.js');
let GPIOIdentification = require('./GPIOIdentification.js');
let I2CDeviceIdentification = require('./I2CDeviceIdentification.js');
let I2CData = require('./I2CData.js');
let EthernetDevice = require('./EthernetDevice.js');
let I2CRegisterAddressSize = require('./I2CRegisterAddressSize.js');
let I2CReadParameter = require('./I2CReadParameter.js');
let GPIOIdentifier = require('./GPIOIdentifier.js');
let GPIOPull = require('./GPIOPull.js');
let I2CDevice = require('./I2CDevice.js');
let EthernetConfiguration = require('./EthernetConfiguration.js');
let EthernetDeviceIdentification = require('./EthernetDeviceIdentification.js');
let InterconnectConfig_GPIOConfiguration = require('./InterconnectConfig_GPIOConfiguration.js');
let InterconnectConfig_ServiceVersion = require('./InterconnectConfig_ServiceVersion.js');
let I2CConfiguration = require('./I2CConfiguration.js');
let I2CWriteParameter = require('./I2CWriteParameter.js');
let GPIOMode = require('./GPIOMode.js');
let GPIOState = require('./GPIOState.js');
let EthernetSpeed = require('./EthernetSpeed.js');
let I2CDeviceAddressing = require('./I2CDeviceAddressing.js');
let I2CWriteRegisterParameter = require('./I2CWriteRegisterParameter.js');
let UARTPortId = require('./UARTPortId.js');
let EthernetDuplex = require('./EthernetDuplex.js');
let I2CReadRegisterParameter = require('./I2CReadRegisterParameter.js');
let GPIOValue = require('./GPIOValue.js');
let InterconnectCyclic_CustomData = require('./InterconnectCyclic_CustomData.js');
let InterconnectCyclic_Feedback = require('./InterconnectCyclic_Feedback.js');
let InterconnectCyclic_CustomData_tool_customData = require('./InterconnectCyclic_CustomData_tool_customData.js');
let InterconnectCyclic_MessageId = require('./InterconnectCyclic_MessageId.js');
let InterconnectCyclic_Feedback_tool_feedback = require('./InterconnectCyclic_Feedback_tool_feedback.js');
let InterconnectCyclic_Command = require('./InterconnectCyclic_Command.js');
let InterconnectCyclic_Command_tool_command = require('./InterconnectCyclic_Command_tool_command.js');
let InterconnectCyclic_ServiceVersion = require('./InterconnectCyclic_ServiceVersion.js');
let ModelId = require('./ModelId.js');
let InterfaceModuleType = require('./InterfaceModuleType.js');
let CompleteProductConfiguration = require('./CompleteProductConfiguration.js');
let VisionModuleType = require('./VisionModuleType.js');
let EndEffectorType = require('./EndEffectorType.js');
let WristType = require('./WristType.js');
let ArmLaterality = require('./ArmLaterality.js');
let ProductConfigurationEndEffectorType = require('./ProductConfigurationEndEffectorType.js');
let BrakeType = require('./BrakeType.js');
let BaseType = require('./BaseType.js');
let Option = require('./Option.js');
let BitRate = require('./BitRate.js');
let IntrinsicProfileIdentifier = require('./IntrinsicProfileIdentifier.js');
let SensorFocusAction_action_parameters = require('./SensorFocusAction_action_parameters.js');
let SensorIdentifier = require('./SensorIdentifier.js');
let OptionIdentifier = require('./OptionIdentifier.js');
let FocusPoint = require('./FocusPoint.js');
let ManualFocus = require('./ManualFocus.js');
let FocusAction = require('./FocusAction.js');
let ExtrinsicParameters = require('./ExtrinsicParameters.js');
let VisionNotification = require('./VisionNotification.js');
let IntrinsicParameters = require('./IntrinsicParameters.js');
let SensorFocusAction = require('./SensorFocusAction.js');
let DistortionCoefficients = require('./DistortionCoefficients.js');
let TranslationVector = require('./TranslationVector.js');
let VisionEvent = require('./VisionEvent.js');
let OptionInformation = require('./OptionInformation.js');
let VisionConfig_ServiceVersion = require('./VisionConfig_ServiceVersion.js');
let Resolution = require('./Resolution.js');
let SensorSettings = require('./SensorSettings.js');
let OptionValue = require('./OptionValue.js');
let FrameRate = require('./FrameRate.js');
let VisionConfig_RotationMatrix = require('./VisionConfig_RotationMatrix.js');
let VisionConfig_RotationMatrixRow = require('./VisionConfig_RotationMatrixRow.js');
let Sensor = require('./Sensor.js');
let FollowCartesianTrajectoryActionResult = require('./FollowCartesianTrajectoryActionResult.js');
let FollowCartesianTrajectoryFeedback = require('./FollowCartesianTrajectoryFeedback.js');
let FollowCartesianTrajectoryActionGoal = require('./FollowCartesianTrajectoryActionGoal.js');
let FollowCartesianTrajectoryGoal = require('./FollowCartesianTrajectoryGoal.js');
let FollowCartesianTrajectoryActionFeedback = require('./FollowCartesianTrajectoryActionFeedback.js');
let FollowCartesianTrajectoryAction = require('./FollowCartesianTrajectoryAction.js');
let FollowCartesianTrajectoryResult = require('./FollowCartesianTrajectoryResult.js');

module.exports = {
  KortexError: KortexError,
  ApiOptions: ApiOptions,
  ErrorCodes: ErrorCodes,
  SubErrorCodes: SubErrorCodes,
  EncoderDerivativeParameters: EncoderDerivativeParameters,
  LoopSelection: LoopSelection,
  ActuatorConfig_ServiceVersion: ActuatorConfig_ServiceVersion,
  ControlLoopParameters: ControlLoopParameters,
  CustomDataSelection: CustomDataSelection,
  SafetyIdentifierBankA: SafetyIdentifierBankA,
  ControlLoopSelection: ControlLoopSelection,
  VectorDriveParameters: VectorDriveParameters,
  ActuatorConfig_SafetyLimitType: ActuatorConfig_SafetyLimitType,
  PositionCommand: PositionCommand,
  CoggingFeedforwardModeInformation: CoggingFeedforwardModeInformation,
  Servoing: Servoing,
  CoggingFeedforwardMode: CoggingFeedforwardMode,
  TorqueOffset: TorqueOffset,
  ActuatorConfig_ControlMode: ActuatorConfig_ControlMode,
  StepResponse: StepResponse,
  CommandModeInformation: CommandModeInformation,
  TorqueCalibration: TorqueCalibration,
  AxisOffsets: AxisOffsets,
  ControlLoop: ControlLoop,
  RampResponse: RampResponse,
  AxisPosition: AxisPosition,
  ActuatorConfig_ControlModeInformation: ActuatorConfig_ControlModeInformation,
  FrequencyResponse: FrequencyResponse,
  CommandMode: CommandMode,
  CustomDataIndex: CustomDataIndex,
  ActuatorCyclic_CustomData: ActuatorCyclic_CustomData,
  ActuatorCyclic_Feedback: ActuatorCyclic_Feedback,
  ActuatorCyclic_Command: ActuatorCyclic_Command,
  StatusFlags: StatusFlags,
  ActuatorCyclic_ServiceVersion: ActuatorCyclic_ServiceVersion,
  CommandFlags: CommandFlags,
  ActuatorCyclic_MessageId: ActuatorCyclic_MessageId,
  ProtectionZone: ProtectionZone,
  Wrench: Wrench,
  IPv4Configuration: IPv4Configuration,
  GpioAction: GpioAction,
  WifiConfigurationList: WifiConfigurationList,
  GpioCommand: GpioCommand,
  NetworkNotification: NetworkNotification,
  Base_ControlModeInformation: Base_ControlModeInformation,
  BridgeResult: BridgeResult,
  ConstrainedJointAngles: ConstrainedJointAngles,
  MappingInfoNotification: MappingInfoNotification,
  AngularWaypoint: AngularWaypoint,
  JointTorques: JointTorques,
  ArmStateInformation: ArmStateInformation,
  SequenceInfoNotificationList: SequenceInfoNotificationList,
  GripperRequest: GripperRequest,
  FullUserProfile: FullUserProfile,
  ShapeType: ShapeType,
  WrenchLimitation: WrenchLimitation,
  TrajectoryErrorType: TrajectoryErrorType,
  ConfigurationNotificationEvent: ConfigurationNotificationEvent,
  Base_RotationMatrixRow: Base_RotationMatrixRow,
  JointNavigationDirection: JointNavigationDirection,
  MapElement: MapElement,
  WifiSecurityType: WifiSecurityType,
  MapEvent_events: MapEvent_events,
  NetworkEvent: NetworkEvent,
  SequenceInformation: SequenceInformation,
  BackupEvent: BackupEvent,
  ControllerInputType: ControllerInputType,
  ConstrainedOrientation: ConstrainedOrientation,
  ProtectionZoneInformation: ProtectionZoneInformation,
  SequenceTasksRange: SequenceTasksRange,
  JointTorque: JointTorque,
  ControllerNotificationList: ControllerNotificationList,
  ControllerEventType: ControllerEventType,
  FirmwareComponentVersion: FirmwareComponentVersion,
  Point: Point,
  GpioBehavior: GpioBehavior,
  ProtectionZoneNotificationList: ProtectionZoneNotificationList,
  ControllerNotification_state: ControllerNotification_state,
  ActionType: ActionType,
  LimitationType: LimitationType,
  BridgeStatus: BridgeStatus,
  EventIdSequenceInfoNotification: EventIdSequenceInfoNotification,
  WifiEnableState: WifiEnableState,
  UserProfileList: UserProfileList,
  SnapshotType: SnapshotType,
  GripperMode: GripperMode,
  SequenceTasksConfiguration: SequenceTasksConfiguration,
  ServoingModeNotification: ServoingModeNotification,
  Admittance: Admittance,
  TrajectoryErrorElement: TrajectoryErrorElement,
  MapGroup: MapGroup,
  MapList: MapList,
  ActionHandle: ActionHandle,
  GpioConfigurationList: GpioConfigurationList,
  EmergencyStop: EmergencyStop,
  WristDigitalInputIdentifier: WristDigitalInputIdentifier,
  GpioPinConfiguration: GpioPinConfiguration,
  Ssid: Ssid,
  PreComputedJointTrajectoryElement: PreComputedJointTrajectoryElement,
  FactoryEvent: FactoryEvent,
  CartesianTrajectoryConstraint: CartesianTrajectoryConstraint,
  JointAngle: JointAngle,
  Action: Action,
  TransformationRow: TransformationRow,
  ChangeJointSpeeds: ChangeJointSpeeds,
  RFConfiguration: RFConfiguration,
  ControllerHandle: ControllerHandle,
  ControllerBehavior: ControllerBehavior,
  KinematicTrajectoryConstraints: KinematicTrajectoryConstraints,
  BridgePortConfig: BridgePortConfig,
  ActionNotificationList: ActionNotificationList,
  CartesianTrajectoryConstraint_type: CartesianTrajectoryConstraint_type,
  MapGroupHandle: MapGroupHandle,
  Timeout: Timeout,
  Action_action_parameters: Action_action_parameters,
  FullIPv4Configuration: FullIPv4Configuration,
  UserEvent: UserEvent,
  GpioPinPropertyFlags: GpioPinPropertyFlags,
  JointsLimitationsList: JointsLimitationsList,
  UserProfile: UserProfile,
  JointAngles: JointAngles,
  ActionNotification: ActionNotification,
  Base_RotationMatrix: Base_RotationMatrix,
  FactoryNotification: FactoryNotification,
  CommunicationInterfaceConfiguration: CommunicationInterfaceConfiguration,
  OperatingMode: OperatingMode,
  GripperCommand: GripperCommand,
  Map: Map,
  SequenceTaskHandle: SequenceTaskHandle,
  ControllerNotification: ControllerNotification,
  SwitchControlMapping: SwitchControlMapping,
  WifiConfiguration: WifiConfiguration,
  WrenchMode: WrenchMode,
  Base_CapSenseConfig: Base_CapSenseConfig,
  SequenceTasks: SequenceTasks,
  ServoingModeNotificationList: ServoingModeNotificationList,
  Mapping: Mapping,
  WaypointList: WaypointList,
  SequenceTaskConfiguration: SequenceTaskConfiguration,
  CartesianWaypoint: CartesianWaypoint,
  WaypointValidationReport: WaypointValidationReport,
  SequenceTasksPair: SequenceTasksPair,
  BridgeList: BridgeList,
  NetworkNotificationList: NetworkNotificationList,
  ProtectionZoneHandle: ProtectionZoneHandle,
  UserNotificationList: UserNotificationList,
  ControllerConfigurationList: ControllerConfigurationList,
  Base_ControlModeNotification: Base_ControlModeNotification,
  SoundType: SoundType,
  ControlModeNotificationList: ControlModeNotificationList,
  SequenceTask: SequenceTask,
  Pose: Pose,
  ConstrainedJointAngle: ConstrainedJointAngle,
  MapGroupList: MapGroupList,
  IKData: IKData,
  BridgeConfig: BridgeConfig,
  BluetoothEnableState: BluetoothEnableState,
  Sequence: Sequence,
  OperatingModeNotificationList: OperatingModeNotificationList,
  ControllerType: ControllerType,
  TrajectoryContinuityMode: TrajectoryContinuityMode,
  RobotEventNotificationList: RobotEventNotificationList,
  PasswordChange: PasswordChange,
  JointSpeed: JointSpeed,
  ActivateMapHandle: ActivateMapHandle,
  ChangeWrench: ChangeWrench,
  LedState: LedState,
  Base_JointSpeeds: Base_JointSpeeds,
  ProtectionZoneList: ProtectionZoneList,
  GpioEvent: GpioEvent,
  TransformationMatrix: TransformationMatrix,
  ConstrainedPosition: ConstrainedPosition,
  Orientation: Orientation,
  TrajectoryErrorIdentifier: TrajectoryErrorIdentifier,
  MapEvent: MapEvent,
  Snapshot: Snapshot,
  Gen3GpioPinId: Gen3GpioPinId,
  TrajectoryErrorReport: TrajectoryErrorReport,
  NetworkType: NetworkType,
  ServoingModeInformation: ServoingModeInformation,
  TwistCommand: TwistCommand,
  ChangeTwist: ChangeTwist,
  SequenceInfoNotification: SequenceInfoNotification,
  Gripper: Gripper,
  Base_ControlMode: Base_ControlMode,
  SafetyEvent: SafetyEvent,
  FirmwareBundleVersions: FirmwareBundleVersions,
  Base_Position: Base_Position,
  WifiInformation: WifiInformation,
  SafetyNotificationList: SafetyNotificationList,
  ArmStateNotification: ArmStateNotification,
  ControllerElementHandle: ControllerElementHandle,
  Query: Query,
  Base_Stop: Base_Stop,
  AppendActionInformation: AppendActionInformation,
  NavigationDirection: NavigationDirection,
  UserList: UserList,
  Waypoint_type_of_waypoint: Waypoint_type_of_waypoint,
  MappingInfoNotificationList: MappingInfoNotificationList,
  MapHandle: MapHandle,
  CartesianLimitationList: CartesianLimitationList,
  IPv4Information: IPv4Information,
  ControllerEvent: ControllerEvent,
  NetworkHandle: NetworkHandle,
  RobotEventNotification: RobotEventNotification,
  ControllerElementEventType: ControllerElementEventType,
  Waypoint: Waypoint,
  WifiEncryptionType: WifiEncryptionType,
  BridgeType: BridgeType,
  Base_ServiceVersion: Base_ServiceVersion,
  SequenceHandle: SequenceHandle,
  ProtectionZoneNotification: ProtectionZoneNotification,
  Xbox360DigitalInputIdentifier: Xbox360DigitalInputIdentifier,
  WrenchCommand: WrenchCommand,
  TrajectoryInfo: TrajectoryInfo,
  AdmittanceMode: AdmittanceMode,
  ControllerConfigurationMode: ControllerConfigurationMode,
  SignalQuality: SignalQuality,
  CartesianLimitation: CartesianLimitation,
  OperatingModeNotification: OperatingModeNotification,
  Faults: Faults,
  ActionEvent: ActionEvent,
  TwistLimitation: TwistLimitation,
  ControllerElementHandle_identifier: ControllerElementHandle_identifier,
  Delay: Delay,
  UserNotification: UserNotification,
  ZoneShape: ZoneShape,
  ConfigurationChangeNotification: ConfigurationChangeNotification,
  JointTrajectoryConstraint: JointTrajectoryConstraint,
  ControllerElementState: ControllerElementState,
  ActionExecutionState: ActionExecutionState,
  ConfigurationChangeNotificationList: ConfigurationChangeNotificationList,
  ConfigurationChangeNotification_configuration_change: ConfigurationChangeNotification_configuration_change,
  BridgeIdentifier: BridgeIdentifier,
  CartesianSpeed: CartesianSpeed,
  Base_SafetyIdentifier: Base_SafetyIdentifier,
  ActuatorInformation: ActuatorInformation,
  JointLimitation: JointLimitation,
  MappingList: MappingList,
  Xbox360AnalogInputIdentifier: Xbox360AnalogInputIdentifier,
  JointTrajectoryConstraintType: JointTrajectoryConstraintType,
  ControllerConfiguration: ControllerConfiguration,
  PreComputedJointTrajectory: PreComputedJointTrajectory,
  RequestedActionType: RequestedActionType,
  Base_GpioConfiguration: Base_GpioConfiguration,
  Finger: Finger,
  ControllerState: ControllerState,
  MappingHandle: MappingHandle,
  ConstrainedPose: ConstrainedPose,
  ControllerList: ControllerList,
  SequenceList: SequenceList,
  SystemTime: SystemTime,
  WifiInformationList: WifiInformationList,
  TrajectoryInfoType: TrajectoryInfoType,
  ServoingMode: ServoingMode,
  ProtectionZoneEvent: ProtectionZoneEvent,
  Twist: Twist,
  ActionList: ActionList,
  AdvancedSequenceHandle: AdvancedSequenceHandle,
  Base_CapSenseMode: Base_CapSenseMode,
  OperatingModeInformation: OperatingModeInformation,
  RobotEvent: RobotEvent,
  ActuatorFeedback: ActuatorFeedback,
  BaseCyclic_Command: BaseCyclic_Command,
  BaseFeedback: BaseFeedback,
  ActuatorCommand: ActuatorCommand,
  ActuatorCustomData: ActuatorCustomData,
  BaseCyclic_Feedback: BaseCyclic_Feedback,
  BaseCyclic_ServiceVersion: BaseCyclic_ServiceVersion,
  BaseCyclic_CustomData: BaseCyclic_CustomData,
  NotificationHandle: NotificationHandle,
  UARTParity: UARTParity,
  Permission: Permission,
  NotificationOptions: NotificationOptions,
  UARTStopBits: UARTStopBits,
  CountryCode: CountryCode,
  UARTSpeed: UARTSpeed,
  Timestamp: Timestamp,
  DeviceHandle: DeviceHandle,
  CartesianReferenceFrame: CartesianReferenceFrame,
  Empty: Empty,
  DeviceTypes: DeviceTypes,
  SafetyNotification: SafetyNotification,
  SafetyHandle: SafetyHandle,
  ArmState: ArmState,
  UserProfileHandle: UserProfileHandle,
  UARTDeviceIdentification: UARTDeviceIdentification,
  SafetyStatusValue: SafetyStatusValue,
  NotificationType: NotificationType,
  Connection: Connection,
  UARTWordLength: UARTWordLength,
  CountryCodeIdentifier: CountryCodeIdentifier,
  Unit: Unit,
  UARTConfiguration: UARTConfiguration,
  LinearTwist: LinearTwist,
  ControlConfig_ControlMode: ControlConfig_ControlMode,
  JointSpeedSoftLimits: JointSpeedSoftLimits,
  DesiredSpeeds: DesiredSpeeds,
  ControlConfig_ServiceVersion: ControlConfig_ServiceVersion,
  AngularTwist: AngularTwist,
  KinematicLimits: KinematicLimits,
  TwistAngularSoftLimit: TwistAngularSoftLimit,
  CartesianTransform: CartesianTransform,
  ControlConfig_JointSpeeds: ControlConfig_JointSpeeds,
  JointAccelerationSoftLimits: JointAccelerationSoftLimits,
  CartesianReferenceFrameInfo: CartesianReferenceFrameInfo,
  ToolConfiguration: ToolConfiguration,
  ControlConfigurationNotification: ControlConfigurationNotification,
  TwistLinearSoftLimit: TwistLinearSoftLimit,
  PayloadInformation: PayloadInformation,
  GravityVector: GravityVector,
  ControlConfig_Position: ControlConfig_Position,
  KinematicLimitsList: KinematicLimitsList,
  ControlConfig_ControlModeInformation: ControlConfig_ControlModeInformation,
  ControlConfigurationEvent: ControlConfigurationEvent,
  ControlConfig_ControlModeNotification: ControlConfig_ControlModeNotification,
  PartNumber: PartNumber,
  SerialNumber: SerialNumber,
  CalibrationElement: CalibrationElement,
  RebootRqst: RebootRqst,
  DeviceConfig_CapSenseConfig: DeviceConfig_CapSenseConfig,
  CalibrationParameter_value: CalibrationParameter_value,
  SafetyInformationList: SafetyInformationList,
  PowerOnSelfTestResult: PowerOnSelfTestResult,
  CalibrationStatus: CalibrationStatus,
  PartNumberRevision: PartNumberRevision,
  SafetyInformation: SafetyInformation,
  DeviceConfig_CapSenseMode: DeviceConfig_CapSenseMode,
  DeviceType: DeviceType,
  MACAddress: MACAddress,
  CapSenseRegister: CapSenseRegister,
  CalibrationParameter: CalibrationParameter,
  SafetyConfiguration: SafetyConfiguration,
  FirmwareVersion: FirmwareVersion,
  BootloaderVersion: BootloaderVersion,
  SafetyEnable: SafetyEnable,
  DeviceConfig_SafetyLimitType: DeviceConfig_SafetyLimitType,
  DeviceConfig_ServiceVersion: DeviceConfig_ServiceVersion,
  IPv4Settings: IPv4Settings,
  Calibration: Calibration,
  RunModes: RunModes,
  ModelNumber: ModelNumber,
  SafetyThreshold: SafetyThreshold,
  SafetyConfigurationList: SafetyConfigurationList,
  CalibrationResult: CalibrationResult,
  CalibrationItem: CalibrationItem,
  SafetyStatus: SafetyStatus,
  RunMode: RunMode,
  DeviceManager_ServiceVersion: DeviceManager_ServiceVersion,
  DeviceHandles: DeviceHandles,
  GripperConfig_SafetyIdentifier: GripperConfig_SafetyIdentifier,
  RobotiqGripperStatusFlags: RobotiqGripperStatusFlags,
  GripperCyclic_Feedback: GripperCyclic_Feedback,
  MotorFeedback: MotorFeedback,
  GripperCyclic_MessageId: GripperCyclic_MessageId,
  GripperCyclic_ServiceVersion: GripperCyclic_ServiceVersion,
  MotorCommand: MotorCommand,
  CustomDataUnit: CustomDataUnit,
  GripperCyclic_CustomData: GripperCyclic_CustomData,
  GripperCyclic_Command: GripperCyclic_Command,
  InterconnectConfig_SafetyIdentifier: InterconnectConfig_SafetyIdentifier,
  I2CMode: I2CMode,
  GPIOIdentification: GPIOIdentification,
  I2CDeviceIdentification: I2CDeviceIdentification,
  I2CData: I2CData,
  EthernetDevice: EthernetDevice,
  I2CRegisterAddressSize: I2CRegisterAddressSize,
  I2CReadParameter: I2CReadParameter,
  GPIOIdentifier: GPIOIdentifier,
  GPIOPull: GPIOPull,
  I2CDevice: I2CDevice,
  EthernetConfiguration: EthernetConfiguration,
  EthernetDeviceIdentification: EthernetDeviceIdentification,
  InterconnectConfig_GPIOConfiguration: InterconnectConfig_GPIOConfiguration,
  InterconnectConfig_ServiceVersion: InterconnectConfig_ServiceVersion,
  I2CConfiguration: I2CConfiguration,
  I2CWriteParameter: I2CWriteParameter,
  GPIOMode: GPIOMode,
  GPIOState: GPIOState,
  EthernetSpeed: EthernetSpeed,
  I2CDeviceAddressing: I2CDeviceAddressing,
  I2CWriteRegisterParameter: I2CWriteRegisterParameter,
  UARTPortId: UARTPortId,
  EthernetDuplex: EthernetDuplex,
  I2CReadRegisterParameter: I2CReadRegisterParameter,
  GPIOValue: GPIOValue,
  InterconnectCyclic_CustomData: InterconnectCyclic_CustomData,
  InterconnectCyclic_Feedback: InterconnectCyclic_Feedback,
  InterconnectCyclic_CustomData_tool_customData: InterconnectCyclic_CustomData_tool_customData,
  InterconnectCyclic_MessageId: InterconnectCyclic_MessageId,
  InterconnectCyclic_Feedback_tool_feedback: InterconnectCyclic_Feedback_tool_feedback,
  InterconnectCyclic_Command: InterconnectCyclic_Command,
  InterconnectCyclic_Command_tool_command: InterconnectCyclic_Command_tool_command,
  InterconnectCyclic_ServiceVersion: InterconnectCyclic_ServiceVersion,
  ModelId: ModelId,
  InterfaceModuleType: InterfaceModuleType,
  CompleteProductConfiguration: CompleteProductConfiguration,
  VisionModuleType: VisionModuleType,
  EndEffectorType: EndEffectorType,
  WristType: WristType,
  ArmLaterality: ArmLaterality,
  ProductConfigurationEndEffectorType: ProductConfigurationEndEffectorType,
  BrakeType: BrakeType,
  BaseType: BaseType,
  Option: Option,
  BitRate: BitRate,
  IntrinsicProfileIdentifier: IntrinsicProfileIdentifier,
  SensorFocusAction_action_parameters: SensorFocusAction_action_parameters,
  SensorIdentifier: SensorIdentifier,
  OptionIdentifier: OptionIdentifier,
  FocusPoint: FocusPoint,
  ManualFocus: ManualFocus,
  FocusAction: FocusAction,
  ExtrinsicParameters: ExtrinsicParameters,
  VisionNotification: VisionNotification,
  IntrinsicParameters: IntrinsicParameters,
  SensorFocusAction: SensorFocusAction,
  DistortionCoefficients: DistortionCoefficients,
  TranslationVector: TranslationVector,
  VisionEvent: VisionEvent,
  OptionInformation: OptionInformation,
  VisionConfig_ServiceVersion: VisionConfig_ServiceVersion,
  Resolution: Resolution,
  SensorSettings: SensorSettings,
  OptionValue: OptionValue,
  FrameRate: FrameRate,
  VisionConfig_RotationMatrix: VisionConfig_RotationMatrix,
  VisionConfig_RotationMatrixRow: VisionConfig_RotationMatrixRow,
  Sensor: Sensor,
  FollowCartesianTrajectoryActionResult: FollowCartesianTrajectoryActionResult,
  FollowCartesianTrajectoryFeedback: FollowCartesianTrajectoryFeedback,
  FollowCartesianTrajectoryActionGoal: FollowCartesianTrajectoryActionGoal,
  FollowCartesianTrajectoryGoal: FollowCartesianTrajectoryGoal,
  FollowCartesianTrajectoryActionFeedback: FollowCartesianTrajectoryActionFeedback,
  FollowCartesianTrajectoryAction: FollowCartesianTrajectoryAction,
  FollowCartesianTrajectoryResult: FollowCartesianTrajectoryResult,
};
