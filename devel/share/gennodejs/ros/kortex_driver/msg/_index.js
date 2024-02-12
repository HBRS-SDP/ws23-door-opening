
"use strict";

let ApiOptions = require('./ApiOptions.js');
let KortexError = require('./KortexError.js');
let ErrorCodes = require('./ErrorCodes.js');
let SubErrorCodes = require('./SubErrorCodes.js');
let FrequencyResponse = require('./FrequencyResponse.js');
let CoggingFeedforwardMode = require('./CoggingFeedforwardMode.js');
let ActuatorConfig_ControlMode = require('./ActuatorConfig_ControlMode.js');
let ActuatorConfig_SafetyLimitType = require('./ActuatorConfig_SafetyLimitType.js');
let CommandModeInformation = require('./CommandModeInformation.js');
let AxisOffsets = require('./AxisOffsets.js');
let ControlLoopSelection = require('./ControlLoopSelection.js');
let SafetyIdentifierBankA = require('./SafetyIdentifierBankA.js');
let CommandMode = require('./CommandMode.js');
let ControlLoopParameters = require('./ControlLoopParameters.js');
let Servoing = require('./Servoing.js');
let CustomDataSelection = require('./CustomDataSelection.js');
let PositionCommand = require('./PositionCommand.js');
let LoopSelection = require('./LoopSelection.js');
let ActuatorConfig_ServiceVersion = require('./ActuatorConfig_ServiceVersion.js');
let ControlLoop = require('./ControlLoop.js');
let AxisPosition = require('./AxisPosition.js');
let CustomDataIndex = require('./CustomDataIndex.js');
let TorqueCalibration = require('./TorqueCalibration.js');
let CoggingFeedforwardModeInformation = require('./CoggingFeedforwardModeInformation.js');
let RampResponse = require('./RampResponse.js');
let StepResponse = require('./StepResponse.js');
let TorqueOffset = require('./TorqueOffset.js');
let VectorDriveParameters = require('./VectorDriveParameters.js');
let EncoderDerivativeParameters = require('./EncoderDerivativeParameters.js');
let ActuatorConfig_ControlModeInformation = require('./ActuatorConfig_ControlModeInformation.js');
let ActuatorCyclic_ServiceVersion = require('./ActuatorCyclic_ServiceVersion.js');
let StatusFlags = require('./StatusFlags.js');
let ActuatorCyclic_MessageId = require('./ActuatorCyclic_MessageId.js');
let ActuatorCyclic_Feedback = require('./ActuatorCyclic_Feedback.js');
let ActuatorCyclic_Command = require('./ActuatorCyclic_Command.js');
let CommandFlags = require('./CommandFlags.js');
let ActuatorCyclic_CustomData = require('./ActuatorCyclic_CustomData.js');
let ConfigurationChangeNotificationList = require('./ConfigurationChangeNotificationList.js');
let TrajectoryErrorType = require('./TrajectoryErrorType.js');
let TrajectoryErrorIdentifier = require('./TrajectoryErrorIdentifier.js');
let RequestedActionType = require('./RequestedActionType.js');
let JointTrajectoryConstraint = require('./JointTrajectoryConstraint.js');
let IKData = require('./IKData.js');
let ActionNotificationList = require('./ActionNotificationList.js');
let Sequence = require('./Sequence.js');
let Delay = require('./Delay.js');
let ActionType = require('./ActionType.js');
let TrajectoryErrorReport = require('./TrajectoryErrorReport.js');
let GripperCommand = require('./GripperCommand.js');
let ControllerElementState = require('./ControllerElementState.js');
let TransformationRow = require('./TransformationRow.js');
let ActivateMapHandle = require('./ActivateMapHandle.js');
let IPv4Information = require('./IPv4Information.js');
let TrajectoryErrorElement = require('./TrajectoryErrorElement.js');
let SystemTime = require('./SystemTime.js');
let SequenceTaskHandle = require('./SequenceTaskHandle.js');
let RobotEventNotification = require('./RobotEventNotification.js');
let ControllerNotification = require('./ControllerNotification.js');
let RFConfiguration = require('./RFConfiguration.js');
let SequenceList = require('./SequenceList.js');
let NetworkType = require('./NetworkType.js');
let AngularWaypoint = require('./AngularWaypoint.js');
let ControlModeNotificationList = require('./ControlModeNotificationList.js');
let Point = require('./Point.js');
let NavigationDirection = require('./NavigationDirection.js');
let ControllerHandle = require('./ControllerHandle.js');
let Waypoint = require('./Waypoint.js');
let GpioConfigurationList = require('./GpioConfigurationList.js');
let ChangeJointSpeeds = require('./ChangeJointSpeeds.js');
let Timeout = require('./Timeout.js');
let GpioEvent = require('./GpioEvent.js');
let FirmwareComponentVersion = require('./FirmwareComponentVersion.js');
let SwitchControlMapping = require('./SwitchControlMapping.js');
let ProtectionZoneHandle = require('./ProtectionZoneHandle.js');
let WifiInformation = require('./WifiInformation.js');
let Base_Position = require('./Base_Position.js');
let ShapeType = require('./ShapeType.js');
let Base_JointSpeeds = require('./Base_JointSpeeds.js');
let BridgeConfig = require('./BridgeConfig.js');
let GpioCommand = require('./GpioCommand.js');
let WifiSecurityType = require('./WifiSecurityType.js');
let SafetyEvent = require('./SafetyEvent.js');
let Gripper = require('./Gripper.js');
let GpioAction = require('./GpioAction.js');
let WrenchCommand = require('./WrenchCommand.js');
let TrajectoryInfoType = require('./TrajectoryInfoType.js');
let WrenchMode = require('./WrenchMode.js');
let ConstrainedPose = require('./ConstrainedPose.js');
let MapGroup = require('./MapGroup.js');
let MappingInfoNotification = require('./MappingInfoNotification.js');
let AppendActionInformation = require('./AppendActionInformation.js');
let NetworkNotification = require('./NetworkNotification.js');
let Mapping = require('./Mapping.js');
let SequenceTasksPair = require('./SequenceTasksPair.js');
let NetworkHandle = require('./NetworkHandle.js');
let CartesianTrajectoryConstraint_type = require('./CartesianTrajectoryConstraint_type.js');
let Faults = require('./Faults.js');
let Xbox360AnalogInputIdentifier = require('./Xbox360AnalogInputIdentifier.js');
let MapEvent_events = require('./MapEvent_events.js');
let Gen3GpioPinId = require('./Gen3GpioPinId.js');
let Xbox360DigitalInputIdentifier = require('./Xbox360DigitalInputIdentifier.js');
let Base_SafetyIdentifier = require('./Base_SafetyIdentifier.js');
let ChangeWrench = require('./ChangeWrench.js');
let MappingInfoNotificationList = require('./MappingInfoNotificationList.js');
let Base_ControlMode = require('./Base_ControlMode.js');
let Finger = require('./Finger.js');
let Base_GpioConfiguration = require('./Base_GpioConfiguration.js');
let WaypointValidationReport = require('./WaypointValidationReport.js');
let ServoingModeInformation = require('./ServoingModeInformation.js');
let AdmittanceMode = require('./AdmittanceMode.js');
let BridgeList = require('./BridgeList.js');
let BackupEvent = require('./BackupEvent.js');
let PreComputedJointTrajectory = require('./PreComputedJointTrajectory.js');
let JointLimitation = require('./JointLimitation.js');
let WaypointList = require('./WaypointList.js');
let JointTorques = require('./JointTorques.js');
let JointTorque = require('./JointTorque.js');
let OperatingModeNotificationList = require('./OperatingModeNotificationList.js');
let ArmStateInformation = require('./ArmStateInformation.js');
let TransformationMatrix = require('./TransformationMatrix.js');
let Waypoint_type_of_waypoint = require('./Waypoint_type_of_waypoint.js');
let ControllerElementHandle = require('./ControllerElementHandle.js');
let ActionEvent = require('./ActionEvent.js');
let SequenceTasksRange = require('./SequenceTasksRange.js');
let ControllerEvent = require('./ControllerEvent.js');
let UserNotificationList = require('./UserNotificationList.js');
let Action_action_parameters = require('./Action_action_parameters.js');
let SequenceHandle = require('./SequenceHandle.js');
let JointNavigationDirection = require('./JointNavigationDirection.js');
let ConfigurationNotificationEvent = require('./ConfigurationNotificationEvent.js');
let BridgeStatus = require('./BridgeStatus.js');
let ControllerConfigurationList = require('./ControllerConfigurationList.js');
let BridgeResult = require('./BridgeResult.js');
let JointsLimitationsList = require('./JointsLimitationsList.js');
let OperatingModeNotification = require('./OperatingModeNotification.js');
let MapElement = require('./MapElement.js');
let ZoneShape = require('./ZoneShape.js');
let Base_RotationMatrixRow = require('./Base_RotationMatrixRow.js');
let GripperMode = require('./GripperMode.js');
let ProtectionZoneNotificationList = require('./ProtectionZoneNotificationList.js');
let ControllerBehavior = require('./ControllerBehavior.js');
let GpioPinConfiguration = require('./GpioPinConfiguration.js');
let Map = require('./Map.js');
let SequenceTasks = require('./SequenceTasks.js');
let ServoingModeNotificationList = require('./ServoingModeNotificationList.js');
let PasswordChange = require('./PasswordChange.js');
let Twist = require('./Twist.js');
let Base_Stop = require('./Base_Stop.js');
let Wrench = require('./Wrench.js');
let UserNotification = require('./UserNotification.js');
let Orientation = require('./Orientation.js');
let GpioBehavior = require('./GpioBehavior.js');
let ControllerElementHandle_identifier = require('./ControllerElementHandle_identifier.js');
let WrenchLimitation = require('./WrenchLimitation.js');
let SignalQuality = require('./SignalQuality.js');
let WristDigitalInputIdentifier = require('./WristDigitalInputIdentifier.js');
let FullUserProfile = require('./FullUserProfile.js');
let SequenceTasksConfiguration = require('./SequenceTasksConfiguration.js');
let CartesianLimitation = require('./CartesianLimitation.js');
let ActionNotification = require('./ActionNotification.js');
let Base_CapSenseMode = require('./Base_CapSenseMode.js');
let ActionExecutionState = require('./ActionExecutionState.js');
let CommunicationInterfaceConfiguration = require('./CommunicationInterfaceConfiguration.js');
let TwistLimitation = require('./TwistLimitation.js');
let SequenceInfoNotificationList = require('./SequenceInfoNotificationList.js');
let ServoingModeNotification = require('./ServoingModeNotification.js');
let SequenceTask = require('./SequenceTask.js');
let OperatingModeInformation = require('./OperatingModeInformation.js');
let GripperRequest = require('./GripperRequest.js');
let UserList = require('./UserList.js');
let ActuatorInformation = require('./ActuatorInformation.js');
let SnapshotType = require('./SnapshotType.js');
let FirmwareBundleVersions = require('./FirmwareBundleVersions.js');
let UserEvent = require('./UserEvent.js');
let Base_ControlModeNotification = require('./Base_ControlModeNotification.js');
let ProtectionZone = require('./ProtectionZone.js');
let WifiConfigurationList = require('./WifiConfigurationList.js');
let Base_RotationMatrix = require('./Base_RotationMatrix.js');
let MapGroupHandle = require('./MapGroupHandle.js');
let Snapshot = require('./Snapshot.js');
let ControllerEventType = require('./ControllerEventType.js');
let WifiEncryptionType = require('./WifiEncryptionType.js');
let WifiConfiguration = require('./WifiConfiguration.js');
let FactoryNotification = require('./FactoryNotification.js');
let ConfigurationChangeNotification_configuration_change = require('./ConfigurationChangeNotification_configuration_change.js');
let SoundType = require('./SoundType.js');
let Base_ServiceVersion = require('./Base_ServiceVersion.js');
let CartesianLimitationList = require('./CartesianLimitationList.js');
let ServoingMode = require('./ServoingMode.js');
let ControllerState = require('./ControllerState.js');
let ConstrainedJointAngle = require('./ConstrainedJointAngle.js');
let UserProfileList = require('./UserProfileList.js');
let SafetyNotificationList = require('./SafetyNotificationList.js');
let JointAngles = require('./JointAngles.js');
let BluetoothEnableState = require('./BluetoothEnableState.js');
let Pose = require('./Pose.js');
let ConstrainedOrientation = require('./ConstrainedOrientation.js');
let ConstrainedPosition = require('./ConstrainedPosition.js');
let MappingHandle = require('./MappingHandle.js');
let CartesianSpeed = require('./CartesianSpeed.js');
let Action = require('./Action.js');
let ControllerConfigurationMode = require('./ControllerConfigurationMode.js');
let MapHandle = require('./MapHandle.js');
let RobotEvent = require('./RobotEvent.js');
let EventIdSequenceInfoNotification = require('./EventIdSequenceInfoNotification.js');
let GpioPinPropertyFlags = require('./GpioPinPropertyFlags.js');
let BridgeIdentifier = require('./BridgeIdentifier.js');
let JointTrajectoryConstraintType = require('./JointTrajectoryConstraintType.js');
let ActionHandle = require('./ActionHandle.js');
let Base_CapSenseConfig = require('./Base_CapSenseConfig.js');
let ControllerNotification_state = require('./ControllerNotification_state.js');
let WifiInformationList = require('./WifiInformationList.js');
let ProtectionZoneNotification = require('./ProtectionZoneNotification.js');
let RobotEventNotificationList = require('./RobotEventNotificationList.js');
let ProtectionZoneList = require('./ProtectionZoneList.js');
let ControllerInputType = require('./ControllerInputType.js');
let PreComputedJointTrajectoryElement = require('./PreComputedJointTrajectoryElement.js');
let ControllerElementEventType = require('./ControllerElementEventType.js');
let CartesianTrajectoryConstraint = require('./CartesianTrajectoryConstraint.js');
let FullIPv4Configuration = require('./FullIPv4Configuration.js');
let JointAngle = require('./JointAngle.js');
let LedState = require('./LedState.js');
let KinematicTrajectoryConstraints = require('./KinematicTrajectoryConstraints.js');
let ActionList = require('./ActionList.js');
let SequenceInformation = require('./SequenceInformation.js');
let MapGroupList = require('./MapGroupList.js');
let TrajectoryContinuityMode = require('./TrajectoryContinuityMode.js');
let UserProfile = require('./UserProfile.js');
let SequenceTaskConfiguration = require('./SequenceTaskConfiguration.js');
let ConstrainedJointAngles = require('./ConstrainedJointAngles.js');
let JointSpeed = require('./JointSpeed.js');
let BridgeType = require('./BridgeType.js');
let ControllerList = require('./ControllerList.js');
let WifiEnableState = require('./WifiEnableState.js');
let Admittance = require('./Admittance.js');
let ControllerNotificationList = require('./ControllerNotificationList.js');
let ControllerType = require('./ControllerType.js');
let TrajectoryInfo = require('./TrajectoryInfo.js');
let NetworkEvent = require('./NetworkEvent.js');
let CartesianWaypoint = require('./CartesianWaypoint.js');
let ArmStateNotification = require('./ArmStateNotification.js');
let MapEvent = require('./MapEvent.js');
let LimitationType = require('./LimitationType.js');
let ChangeTwist = require('./ChangeTwist.js');
let ProtectionZoneEvent = require('./ProtectionZoneEvent.js');
let FactoryEvent = require('./FactoryEvent.js');
let NetworkNotificationList = require('./NetworkNotificationList.js');
let EmergencyStop = require('./EmergencyStop.js');
let AdvancedSequenceHandle = require('./AdvancedSequenceHandle.js');
let MapList = require('./MapList.js');
let TwistCommand = require('./TwistCommand.js');
let Base_ControlModeInformation = require('./Base_ControlModeInformation.js');
let Query = require('./Query.js');
let Ssid = require('./Ssid.js');
let IPv4Configuration = require('./IPv4Configuration.js');
let ProtectionZoneInformation = require('./ProtectionZoneInformation.js');
let ConfigurationChangeNotification = require('./ConfigurationChangeNotification.js');
let SequenceInfoNotification = require('./SequenceInfoNotification.js');
let BridgePortConfig = require('./BridgePortConfig.js');
let MappingList = require('./MappingList.js');
let OperatingMode = require('./OperatingMode.js');
let ControllerConfiguration = require('./ControllerConfiguration.js');
let ActuatorFeedback = require('./ActuatorFeedback.js');
let ActuatorCommand = require('./ActuatorCommand.js');
let BaseCyclic_Command = require('./BaseCyclic_Command.js');
let BaseCyclic_ServiceVersion = require('./BaseCyclic_ServiceVersion.js');
let BaseCyclic_Feedback = require('./BaseCyclic_Feedback.js');
let ActuatorCustomData = require('./ActuatorCustomData.js');
let BaseFeedback = require('./BaseFeedback.js');
let BaseCyclic_CustomData = require('./BaseCyclic_CustomData.js');
let NotificationHandle = require('./NotificationHandle.js');
let SafetyNotification = require('./SafetyNotification.js');
let UserProfileHandle = require('./UserProfileHandle.js');
let Empty = require('./Empty.js');
let NotificationOptions = require('./NotificationOptions.js');
let SafetyHandle = require('./SafetyHandle.js');
let UARTDeviceIdentification = require('./UARTDeviceIdentification.js');
let CountryCodeIdentifier = require('./CountryCodeIdentifier.js');
let Permission = require('./Permission.js');
let DeviceTypes = require('./DeviceTypes.js');
let UARTWordLength = require('./UARTWordLength.js');
let UARTConfiguration = require('./UARTConfiguration.js');
let UARTSpeed = require('./UARTSpeed.js');
let Connection = require('./Connection.js');
let Timestamp = require('./Timestamp.js');
let DeviceHandle = require('./DeviceHandle.js');
let SafetyStatusValue = require('./SafetyStatusValue.js');
let UARTParity = require('./UARTParity.js');
let ArmState = require('./ArmState.js');
let NotificationType = require('./NotificationType.js');
let Unit = require('./Unit.js');
let CartesianReferenceFrame = require('./CartesianReferenceFrame.js');
let CountryCode = require('./CountryCode.js');
let UARTStopBits = require('./UARTStopBits.js');
let TwistAngularSoftLimit = require('./TwistAngularSoftLimit.js');
let PayloadInformation = require('./PayloadInformation.js');
let DesiredSpeeds = require('./DesiredSpeeds.js');
let GravityVector = require('./GravityVector.js');
let CartesianReferenceFrameInfo = require('./CartesianReferenceFrameInfo.js');
let ToolConfiguration = require('./ToolConfiguration.js');
let ControlConfig_Position = require('./ControlConfig_Position.js');
let LinearTwist = require('./LinearTwist.js');
let ControlConfigurationNotification = require('./ControlConfigurationNotification.js');
let ControlConfig_ControlMode = require('./ControlConfig_ControlMode.js');
let ControlConfig_ControlModeNotification = require('./ControlConfig_ControlModeNotification.js');
let ControlConfig_JointSpeeds = require('./ControlConfig_JointSpeeds.js');
let CartesianTransform = require('./CartesianTransform.js');
let ControlConfig_ServiceVersion = require('./ControlConfig_ServiceVersion.js');
let ControlConfig_ControlModeInformation = require('./ControlConfig_ControlModeInformation.js');
let ControlConfigurationEvent = require('./ControlConfigurationEvent.js');
let JointSpeedSoftLimits = require('./JointSpeedSoftLimits.js');
let KinematicLimitsList = require('./KinematicLimitsList.js');
let AngularTwist = require('./AngularTwist.js');
let KinematicLimits = require('./KinematicLimits.js');
let TwistLinearSoftLimit = require('./TwistLinearSoftLimit.js');
let JointAccelerationSoftLimits = require('./JointAccelerationSoftLimits.js');
let CalibrationParameter_value = require('./CalibrationParameter_value.js');
let ModelNumber = require('./ModelNumber.js');
let SafetyConfigurationList = require('./SafetyConfigurationList.js');
let MACAddress = require('./MACAddress.js');
let CalibrationStatus = require('./CalibrationStatus.js');
let RebootRqst = require('./RebootRqst.js');
let DeviceConfig_ServiceVersion = require('./DeviceConfig_ServiceVersion.js');
let CalibrationElement = require('./CalibrationElement.js');
let CalibrationItem = require('./CalibrationItem.js');
let RunModes = require('./RunModes.js');
let SafetyConfiguration = require('./SafetyConfiguration.js');
let CalibrationParameter = require('./CalibrationParameter.js');
let SafetyStatus = require('./SafetyStatus.js');
let FirmwareVersion = require('./FirmwareVersion.js');
let SafetyInformation = require('./SafetyInformation.js');
let DeviceConfig_CapSenseMode = require('./DeviceConfig_CapSenseMode.js');
let PartNumberRevision = require('./PartNumberRevision.js');
let SafetyThreshold = require('./SafetyThreshold.js');
let DeviceConfig_SafetyLimitType = require('./DeviceConfig_SafetyLimitType.js');
let SafetyInformationList = require('./SafetyInformationList.js');
let SafetyEnable = require('./SafetyEnable.js');
let PartNumber = require('./PartNumber.js');
let Calibration = require('./Calibration.js');
let PowerOnSelfTestResult = require('./PowerOnSelfTestResult.js');
let CalibrationResult = require('./CalibrationResult.js');
let DeviceConfig_CapSenseConfig = require('./DeviceConfig_CapSenseConfig.js');
let SerialNumber = require('./SerialNumber.js');
let RunMode = require('./RunMode.js');
let BootloaderVersion = require('./BootloaderVersion.js');
let IPv4Settings = require('./IPv4Settings.js');
let DeviceType = require('./DeviceType.js');
let CapSenseRegister = require('./CapSenseRegister.js');
let DeviceManager_ServiceVersion = require('./DeviceManager_ServiceVersion.js');
let DeviceHandles = require('./DeviceHandles.js');
let RobotiqGripperStatusFlags = require('./RobotiqGripperStatusFlags.js');
let GripperConfig_SafetyIdentifier = require('./GripperConfig_SafetyIdentifier.js');
let MotorFeedback = require('./MotorFeedback.js');
let CustomDataUnit = require('./CustomDataUnit.js');
let MotorCommand = require('./MotorCommand.js');
let GripperCyclic_CustomData = require('./GripperCyclic_CustomData.js');
let GripperCyclic_Feedback = require('./GripperCyclic_Feedback.js');
let GripperCyclic_ServiceVersion = require('./GripperCyclic_ServiceVersion.js');
let GripperCyclic_Command = require('./GripperCyclic_Command.js');
let GripperCyclic_MessageId = require('./GripperCyclic_MessageId.js');
let GPIOIdentifier = require('./GPIOIdentifier.js');
let GPIOIdentification = require('./GPIOIdentification.js');
let I2CRegisterAddressSize = require('./I2CRegisterAddressSize.js');
let GPIOValue = require('./GPIOValue.js');
let EthernetDeviceIdentification = require('./EthernetDeviceIdentification.js');
let EthernetDuplex = require('./EthernetDuplex.js');
let EthernetSpeed = require('./EthernetSpeed.js');
let I2CWriteRegisterParameter = require('./I2CWriteRegisterParameter.js');
let GPIOMode = require('./GPIOMode.js');
let EthernetDevice = require('./EthernetDevice.js');
let I2CDeviceAddressing = require('./I2CDeviceAddressing.js');
let I2CDevice = require('./I2CDevice.js');
let I2CMode = require('./I2CMode.js');
let I2CReadParameter = require('./I2CReadParameter.js');
let InterconnectConfig_GPIOConfiguration = require('./InterconnectConfig_GPIOConfiguration.js');
let GPIOState = require('./GPIOState.js');
let I2CData = require('./I2CData.js');
let I2CReadRegisterParameter = require('./I2CReadRegisterParameter.js');
let UARTPortId = require('./UARTPortId.js');
let GPIOPull = require('./GPIOPull.js');
let InterconnectConfig_SafetyIdentifier = require('./InterconnectConfig_SafetyIdentifier.js');
let EthernetConfiguration = require('./EthernetConfiguration.js');
let I2CWriteParameter = require('./I2CWriteParameter.js');
let InterconnectConfig_ServiceVersion = require('./InterconnectConfig_ServiceVersion.js');
let I2CDeviceIdentification = require('./I2CDeviceIdentification.js');
let I2CConfiguration = require('./I2CConfiguration.js');
let InterconnectCyclic_Feedback_tool_feedback = require('./InterconnectCyclic_Feedback_tool_feedback.js');
let InterconnectCyclic_MessageId = require('./InterconnectCyclic_MessageId.js');
let InterconnectCyclic_Feedback = require('./InterconnectCyclic_Feedback.js');
let InterconnectCyclic_Command_tool_command = require('./InterconnectCyclic_Command_tool_command.js');
let InterconnectCyclic_ServiceVersion = require('./InterconnectCyclic_ServiceVersion.js');
let InterconnectCyclic_CustomData = require('./InterconnectCyclic_CustomData.js');
let InterconnectCyclic_CustomData_tool_customData = require('./InterconnectCyclic_CustomData_tool_customData.js');
let InterconnectCyclic_Command = require('./InterconnectCyclic_Command.js');
let ArmLaterality = require('./ArmLaterality.js');
let ModelId = require('./ModelId.js');
let CompleteProductConfiguration = require('./CompleteProductConfiguration.js');
let VisionModuleType = require('./VisionModuleType.js');
let ProductConfigurationEndEffectorType = require('./ProductConfigurationEndEffectorType.js');
let WristType = require('./WristType.js');
let InterfaceModuleType = require('./InterfaceModuleType.js');
let BrakeType = require('./BrakeType.js');
let BaseType = require('./BaseType.js');
let EndEffectorType = require('./EndEffectorType.js');
let VisionNotification = require('./VisionNotification.js');
let TranslationVector = require('./TranslationVector.js');
let BitRate = require('./BitRate.js');
let VisionConfig_ServiceVersion = require('./VisionConfig_ServiceVersion.js');
let VisionEvent = require('./VisionEvent.js');
let SensorFocusAction = require('./SensorFocusAction.js');
let Sensor = require('./Sensor.js');
let Resolution = require('./Resolution.js');
let OptionIdentifier = require('./OptionIdentifier.js');
let FrameRate = require('./FrameRate.js');
let FocusAction = require('./FocusAction.js');
let ExtrinsicParameters = require('./ExtrinsicParameters.js');
let DistortionCoefficients = require('./DistortionCoefficients.js');
let Option = require('./Option.js');
let FocusPoint = require('./FocusPoint.js');
let SensorFocusAction_action_parameters = require('./SensorFocusAction_action_parameters.js');
let OptionValue = require('./OptionValue.js');
let SensorIdentifier = require('./SensorIdentifier.js');
let IntrinsicProfileIdentifier = require('./IntrinsicProfileIdentifier.js');
let IntrinsicParameters = require('./IntrinsicParameters.js');
let SensorSettings = require('./SensorSettings.js');
let VisionConfig_RotationMatrix = require('./VisionConfig_RotationMatrix.js');
let OptionInformation = require('./OptionInformation.js');
let ManualFocus = require('./ManualFocus.js');
let VisionConfig_RotationMatrixRow = require('./VisionConfig_RotationMatrixRow.js');
let FollowCartesianTrajectoryGoal = require('./FollowCartesianTrajectoryGoal.js');
let FollowCartesianTrajectoryFeedback = require('./FollowCartesianTrajectoryFeedback.js');
let FollowCartesianTrajectoryAction = require('./FollowCartesianTrajectoryAction.js');
let FollowCartesianTrajectoryResult = require('./FollowCartesianTrajectoryResult.js');
let FollowCartesianTrajectoryActionResult = require('./FollowCartesianTrajectoryActionResult.js');
let FollowCartesianTrajectoryActionFeedback = require('./FollowCartesianTrajectoryActionFeedback.js');
let FollowCartesianTrajectoryActionGoal = require('./FollowCartesianTrajectoryActionGoal.js');

module.exports = {
  ApiOptions: ApiOptions,
  KortexError: KortexError,
  ErrorCodes: ErrorCodes,
  SubErrorCodes: SubErrorCodes,
  FrequencyResponse: FrequencyResponse,
  CoggingFeedforwardMode: CoggingFeedforwardMode,
  ActuatorConfig_ControlMode: ActuatorConfig_ControlMode,
  ActuatorConfig_SafetyLimitType: ActuatorConfig_SafetyLimitType,
  CommandModeInformation: CommandModeInformation,
  AxisOffsets: AxisOffsets,
  ControlLoopSelection: ControlLoopSelection,
  SafetyIdentifierBankA: SafetyIdentifierBankA,
  CommandMode: CommandMode,
  ControlLoopParameters: ControlLoopParameters,
  Servoing: Servoing,
  CustomDataSelection: CustomDataSelection,
  PositionCommand: PositionCommand,
  LoopSelection: LoopSelection,
  ActuatorConfig_ServiceVersion: ActuatorConfig_ServiceVersion,
  ControlLoop: ControlLoop,
  AxisPosition: AxisPosition,
  CustomDataIndex: CustomDataIndex,
  TorqueCalibration: TorqueCalibration,
  CoggingFeedforwardModeInformation: CoggingFeedforwardModeInformation,
  RampResponse: RampResponse,
  StepResponse: StepResponse,
  TorqueOffset: TorqueOffset,
  VectorDriveParameters: VectorDriveParameters,
  EncoderDerivativeParameters: EncoderDerivativeParameters,
  ActuatorConfig_ControlModeInformation: ActuatorConfig_ControlModeInformation,
  ActuatorCyclic_ServiceVersion: ActuatorCyclic_ServiceVersion,
  StatusFlags: StatusFlags,
  ActuatorCyclic_MessageId: ActuatorCyclic_MessageId,
  ActuatorCyclic_Feedback: ActuatorCyclic_Feedback,
  ActuatorCyclic_Command: ActuatorCyclic_Command,
  CommandFlags: CommandFlags,
  ActuatorCyclic_CustomData: ActuatorCyclic_CustomData,
  ConfigurationChangeNotificationList: ConfigurationChangeNotificationList,
  TrajectoryErrorType: TrajectoryErrorType,
  TrajectoryErrorIdentifier: TrajectoryErrorIdentifier,
  RequestedActionType: RequestedActionType,
  JointTrajectoryConstraint: JointTrajectoryConstraint,
  IKData: IKData,
  ActionNotificationList: ActionNotificationList,
  Sequence: Sequence,
  Delay: Delay,
  ActionType: ActionType,
  TrajectoryErrorReport: TrajectoryErrorReport,
  GripperCommand: GripperCommand,
  ControllerElementState: ControllerElementState,
  TransformationRow: TransformationRow,
  ActivateMapHandle: ActivateMapHandle,
  IPv4Information: IPv4Information,
  TrajectoryErrorElement: TrajectoryErrorElement,
  SystemTime: SystemTime,
  SequenceTaskHandle: SequenceTaskHandle,
  RobotEventNotification: RobotEventNotification,
  ControllerNotification: ControllerNotification,
  RFConfiguration: RFConfiguration,
  SequenceList: SequenceList,
  NetworkType: NetworkType,
  AngularWaypoint: AngularWaypoint,
  ControlModeNotificationList: ControlModeNotificationList,
  Point: Point,
  NavigationDirection: NavigationDirection,
  ControllerHandle: ControllerHandle,
  Waypoint: Waypoint,
  GpioConfigurationList: GpioConfigurationList,
  ChangeJointSpeeds: ChangeJointSpeeds,
  Timeout: Timeout,
  GpioEvent: GpioEvent,
  FirmwareComponentVersion: FirmwareComponentVersion,
  SwitchControlMapping: SwitchControlMapping,
  ProtectionZoneHandle: ProtectionZoneHandle,
  WifiInformation: WifiInformation,
  Base_Position: Base_Position,
  ShapeType: ShapeType,
  Base_JointSpeeds: Base_JointSpeeds,
  BridgeConfig: BridgeConfig,
  GpioCommand: GpioCommand,
  WifiSecurityType: WifiSecurityType,
  SafetyEvent: SafetyEvent,
  Gripper: Gripper,
  GpioAction: GpioAction,
  WrenchCommand: WrenchCommand,
  TrajectoryInfoType: TrajectoryInfoType,
  WrenchMode: WrenchMode,
  ConstrainedPose: ConstrainedPose,
  MapGroup: MapGroup,
  MappingInfoNotification: MappingInfoNotification,
  AppendActionInformation: AppendActionInformation,
  NetworkNotification: NetworkNotification,
  Mapping: Mapping,
  SequenceTasksPair: SequenceTasksPair,
  NetworkHandle: NetworkHandle,
  CartesianTrajectoryConstraint_type: CartesianTrajectoryConstraint_type,
  Faults: Faults,
  Xbox360AnalogInputIdentifier: Xbox360AnalogInputIdentifier,
  MapEvent_events: MapEvent_events,
  Gen3GpioPinId: Gen3GpioPinId,
  Xbox360DigitalInputIdentifier: Xbox360DigitalInputIdentifier,
  Base_SafetyIdentifier: Base_SafetyIdentifier,
  ChangeWrench: ChangeWrench,
  MappingInfoNotificationList: MappingInfoNotificationList,
  Base_ControlMode: Base_ControlMode,
  Finger: Finger,
  Base_GpioConfiguration: Base_GpioConfiguration,
  WaypointValidationReport: WaypointValidationReport,
  ServoingModeInformation: ServoingModeInformation,
  AdmittanceMode: AdmittanceMode,
  BridgeList: BridgeList,
  BackupEvent: BackupEvent,
  PreComputedJointTrajectory: PreComputedJointTrajectory,
  JointLimitation: JointLimitation,
  WaypointList: WaypointList,
  JointTorques: JointTorques,
  JointTorque: JointTorque,
  OperatingModeNotificationList: OperatingModeNotificationList,
  ArmStateInformation: ArmStateInformation,
  TransformationMatrix: TransformationMatrix,
  Waypoint_type_of_waypoint: Waypoint_type_of_waypoint,
  ControllerElementHandle: ControllerElementHandle,
  ActionEvent: ActionEvent,
  SequenceTasksRange: SequenceTasksRange,
  ControllerEvent: ControllerEvent,
  UserNotificationList: UserNotificationList,
  Action_action_parameters: Action_action_parameters,
  SequenceHandle: SequenceHandle,
  JointNavigationDirection: JointNavigationDirection,
  ConfigurationNotificationEvent: ConfigurationNotificationEvent,
  BridgeStatus: BridgeStatus,
  ControllerConfigurationList: ControllerConfigurationList,
  BridgeResult: BridgeResult,
  JointsLimitationsList: JointsLimitationsList,
  OperatingModeNotification: OperatingModeNotification,
  MapElement: MapElement,
  ZoneShape: ZoneShape,
  Base_RotationMatrixRow: Base_RotationMatrixRow,
  GripperMode: GripperMode,
  ProtectionZoneNotificationList: ProtectionZoneNotificationList,
  ControllerBehavior: ControllerBehavior,
  GpioPinConfiguration: GpioPinConfiguration,
  Map: Map,
  SequenceTasks: SequenceTasks,
  ServoingModeNotificationList: ServoingModeNotificationList,
  PasswordChange: PasswordChange,
  Twist: Twist,
  Base_Stop: Base_Stop,
  Wrench: Wrench,
  UserNotification: UserNotification,
  Orientation: Orientation,
  GpioBehavior: GpioBehavior,
  ControllerElementHandle_identifier: ControllerElementHandle_identifier,
  WrenchLimitation: WrenchLimitation,
  SignalQuality: SignalQuality,
  WristDigitalInputIdentifier: WristDigitalInputIdentifier,
  FullUserProfile: FullUserProfile,
  SequenceTasksConfiguration: SequenceTasksConfiguration,
  CartesianLimitation: CartesianLimitation,
  ActionNotification: ActionNotification,
  Base_CapSenseMode: Base_CapSenseMode,
  ActionExecutionState: ActionExecutionState,
  CommunicationInterfaceConfiguration: CommunicationInterfaceConfiguration,
  TwistLimitation: TwistLimitation,
  SequenceInfoNotificationList: SequenceInfoNotificationList,
  ServoingModeNotification: ServoingModeNotification,
  SequenceTask: SequenceTask,
  OperatingModeInformation: OperatingModeInformation,
  GripperRequest: GripperRequest,
  UserList: UserList,
  ActuatorInformation: ActuatorInformation,
  SnapshotType: SnapshotType,
  FirmwareBundleVersions: FirmwareBundleVersions,
  UserEvent: UserEvent,
  Base_ControlModeNotification: Base_ControlModeNotification,
  ProtectionZone: ProtectionZone,
  WifiConfigurationList: WifiConfigurationList,
  Base_RotationMatrix: Base_RotationMatrix,
  MapGroupHandle: MapGroupHandle,
  Snapshot: Snapshot,
  ControllerEventType: ControllerEventType,
  WifiEncryptionType: WifiEncryptionType,
  WifiConfiguration: WifiConfiguration,
  FactoryNotification: FactoryNotification,
  ConfigurationChangeNotification_configuration_change: ConfigurationChangeNotification_configuration_change,
  SoundType: SoundType,
  Base_ServiceVersion: Base_ServiceVersion,
  CartesianLimitationList: CartesianLimitationList,
  ServoingMode: ServoingMode,
  ControllerState: ControllerState,
  ConstrainedJointAngle: ConstrainedJointAngle,
  UserProfileList: UserProfileList,
  SafetyNotificationList: SafetyNotificationList,
  JointAngles: JointAngles,
  BluetoothEnableState: BluetoothEnableState,
  Pose: Pose,
  ConstrainedOrientation: ConstrainedOrientation,
  ConstrainedPosition: ConstrainedPosition,
  MappingHandle: MappingHandle,
  CartesianSpeed: CartesianSpeed,
  Action: Action,
  ControllerConfigurationMode: ControllerConfigurationMode,
  MapHandle: MapHandle,
  RobotEvent: RobotEvent,
  EventIdSequenceInfoNotification: EventIdSequenceInfoNotification,
  GpioPinPropertyFlags: GpioPinPropertyFlags,
  BridgeIdentifier: BridgeIdentifier,
  JointTrajectoryConstraintType: JointTrajectoryConstraintType,
  ActionHandle: ActionHandle,
  Base_CapSenseConfig: Base_CapSenseConfig,
  ControllerNotification_state: ControllerNotification_state,
  WifiInformationList: WifiInformationList,
  ProtectionZoneNotification: ProtectionZoneNotification,
  RobotEventNotificationList: RobotEventNotificationList,
  ProtectionZoneList: ProtectionZoneList,
  ControllerInputType: ControllerInputType,
  PreComputedJointTrajectoryElement: PreComputedJointTrajectoryElement,
  ControllerElementEventType: ControllerElementEventType,
  CartesianTrajectoryConstraint: CartesianTrajectoryConstraint,
  FullIPv4Configuration: FullIPv4Configuration,
  JointAngle: JointAngle,
  LedState: LedState,
  KinematicTrajectoryConstraints: KinematicTrajectoryConstraints,
  ActionList: ActionList,
  SequenceInformation: SequenceInformation,
  MapGroupList: MapGroupList,
  TrajectoryContinuityMode: TrajectoryContinuityMode,
  UserProfile: UserProfile,
  SequenceTaskConfiguration: SequenceTaskConfiguration,
  ConstrainedJointAngles: ConstrainedJointAngles,
  JointSpeed: JointSpeed,
  BridgeType: BridgeType,
  ControllerList: ControllerList,
  WifiEnableState: WifiEnableState,
  Admittance: Admittance,
  ControllerNotificationList: ControllerNotificationList,
  ControllerType: ControllerType,
  TrajectoryInfo: TrajectoryInfo,
  NetworkEvent: NetworkEvent,
  CartesianWaypoint: CartesianWaypoint,
  ArmStateNotification: ArmStateNotification,
  MapEvent: MapEvent,
  LimitationType: LimitationType,
  ChangeTwist: ChangeTwist,
  ProtectionZoneEvent: ProtectionZoneEvent,
  FactoryEvent: FactoryEvent,
  NetworkNotificationList: NetworkNotificationList,
  EmergencyStop: EmergencyStop,
  AdvancedSequenceHandle: AdvancedSequenceHandle,
  MapList: MapList,
  TwistCommand: TwistCommand,
  Base_ControlModeInformation: Base_ControlModeInformation,
  Query: Query,
  Ssid: Ssid,
  IPv4Configuration: IPv4Configuration,
  ProtectionZoneInformation: ProtectionZoneInformation,
  ConfigurationChangeNotification: ConfigurationChangeNotification,
  SequenceInfoNotification: SequenceInfoNotification,
  BridgePortConfig: BridgePortConfig,
  MappingList: MappingList,
  OperatingMode: OperatingMode,
  ControllerConfiguration: ControllerConfiguration,
  ActuatorFeedback: ActuatorFeedback,
  ActuatorCommand: ActuatorCommand,
  BaseCyclic_Command: BaseCyclic_Command,
  BaseCyclic_ServiceVersion: BaseCyclic_ServiceVersion,
  BaseCyclic_Feedback: BaseCyclic_Feedback,
  ActuatorCustomData: ActuatorCustomData,
  BaseFeedback: BaseFeedback,
  BaseCyclic_CustomData: BaseCyclic_CustomData,
  NotificationHandle: NotificationHandle,
  SafetyNotification: SafetyNotification,
  UserProfileHandle: UserProfileHandle,
  Empty: Empty,
  NotificationOptions: NotificationOptions,
  SafetyHandle: SafetyHandle,
  UARTDeviceIdentification: UARTDeviceIdentification,
  CountryCodeIdentifier: CountryCodeIdentifier,
  Permission: Permission,
  DeviceTypes: DeviceTypes,
  UARTWordLength: UARTWordLength,
  UARTConfiguration: UARTConfiguration,
  UARTSpeed: UARTSpeed,
  Connection: Connection,
  Timestamp: Timestamp,
  DeviceHandle: DeviceHandle,
  SafetyStatusValue: SafetyStatusValue,
  UARTParity: UARTParity,
  ArmState: ArmState,
  NotificationType: NotificationType,
  Unit: Unit,
  CartesianReferenceFrame: CartesianReferenceFrame,
  CountryCode: CountryCode,
  UARTStopBits: UARTStopBits,
  TwistAngularSoftLimit: TwistAngularSoftLimit,
  PayloadInformation: PayloadInformation,
  DesiredSpeeds: DesiredSpeeds,
  GravityVector: GravityVector,
  CartesianReferenceFrameInfo: CartesianReferenceFrameInfo,
  ToolConfiguration: ToolConfiguration,
  ControlConfig_Position: ControlConfig_Position,
  LinearTwist: LinearTwist,
  ControlConfigurationNotification: ControlConfigurationNotification,
  ControlConfig_ControlMode: ControlConfig_ControlMode,
  ControlConfig_ControlModeNotification: ControlConfig_ControlModeNotification,
  ControlConfig_JointSpeeds: ControlConfig_JointSpeeds,
  CartesianTransform: CartesianTransform,
  ControlConfig_ServiceVersion: ControlConfig_ServiceVersion,
  ControlConfig_ControlModeInformation: ControlConfig_ControlModeInformation,
  ControlConfigurationEvent: ControlConfigurationEvent,
  JointSpeedSoftLimits: JointSpeedSoftLimits,
  KinematicLimitsList: KinematicLimitsList,
  AngularTwist: AngularTwist,
  KinematicLimits: KinematicLimits,
  TwistLinearSoftLimit: TwistLinearSoftLimit,
  JointAccelerationSoftLimits: JointAccelerationSoftLimits,
  CalibrationParameter_value: CalibrationParameter_value,
  ModelNumber: ModelNumber,
  SafetyConfigurationList: SafetyConfigurationList,
  MACAddress: MACAddress,
  CalibrationStatus: CalibrationStatus,
  RebootRqst: RebootRqst,
  DeviceConfig_ServiceVersion: DeviceConfig_ServiceVersion,
  CalibrationElement: CalibrationElement,
  CalibrationItem: CalibrationItem,
  RunModes: RunModes,
  SafetyConfiguration: SafetyConfiguration,
  CalibrationParameter: CalibrationParameter,
  SafetyStatus: SafetyStatus,
  FirmwareVersion: FirmwareVersion,
  SafetyInformation: SafetyInformation,
  DeviceConfig_CapSenseMode: DeviceConfig_CapSenseMode,
  PartNumberRevision: PartNumberRevision,
  SafetyThreshold: SafetyThreshold,
  DeviceConfig_SafetyLimitType: DeviceConfig_SafetyLimitType,
  SafetyInformationList: SafetyInformationList,
  SafetyEnable: SafetyEnable,
  PartNumber: PartNumber,
  Calibration: Calibration,
  PowerOnSelfTestResult: PowerOnSelfTestResult,
  CalibrationResult: CalibrationResult,
  DeviceConfig_CapSenseConfig: DeviceConfig_CapSenseConfig,
  SerialNumber: SerialNumber,
  RunMode: RunMode,
  BootloaderVersion: BootloaderVersion,
  IPv4Settings: IPv4Settings,
  DeviceType: DeviceType,
  CapSenseRegister: CapSenseRegister,
  DeviceManager_ServiceVersion: DeviceManager_ServiceVersion,
  DeviceHandles: DeviceHandles,
  RobotiqGripperStatusFlags: RobotiqGripperStatusFlags,
  GripperConfig_SafetyIdentifier: GripperConfig_SafetyIdentifier,
  MotorFeedback: MotorFeedback,
  CustomDataUnit: CustomDataUnit,
  MotorCommand: MotorCommand,
  GripperCyclic_CustomData: GripperCyclic_CustomData,
  GripperCyclic_Feedback: GripperCyclic_Feedback,
  GripperCyclic_ServiceVersion: GripperCyclic_ServiceVersion,
  GripperCyclic_Command: GripperCyclic_Command,
  GripperCyclic_MessageId: GripperCyclic_MessageId,
  GPIOIdentifier: GPIOIdentifier,
  GPIOIdentification: GPIOIdentification,
  I2CRegisterAddressSize: I2CRegisterAddressSize,
  GPIOValue: GPIOValue,
  EthernetDeviceIdentification: EthernetDeviceIdentification,
  EthernetDuplex: EthernetDuplex,
  EthernetSpeed: EthernetSpeed,
  I2CWriteRegisterParameter: I2CWriteRegisterParameter,
  GPIOMode: GPIOMode,
  EthernetDevice: EthernetDevice,
  I2CDeviceAddressing: I2CDeviceAddressing,
  I2CDevice: I2CDevice,
  I2CMode: I2CMode,
  I2CReadParameter: I2CReadParameter,
  InterconnectConfig_GPIOConfiguration: InterconnectConfig_GPIOConfiguration,
  GPIOState: GPIOState,
  I2CData: I2CData,
  I2CReadRegisterParameter: I2CReadRegisterParameter,
  UARTPortId: UARTPortId,
  GPIOPull: GPIOPull,
  InterconnectConfig_SafetyIdentifier: InterconnectConfig_SafetyIdentifier,
  EthernetConfiguration: EthernetConfiguration,
  I2CWriteParameter: I2CWriteParameter,
  InterconnectConfig_ServiceVersion: InterconnectConfig_ServiceVersion,
  I2CDeviceIdentification: I2CDeviceIdentification,
  I2CConfiguration: I2CConfiguration,
  InterconnectCyclic_Feedback_tool_feedback: InterconnectCyclic_Feedback_tool_feedback,
  InterconnectCyclic_MessageId: InterconnectCyclic_MessageId,
  InterconnectCyclic_Feedback: InterconnectCyclic_Feedback,
  InterconnectCyclic_Command_tool_command: InterconnectCyclic_Command_tool_command,
  InterconnectCyclic_ServiceVersion: InterconnectCyclic_ServiceVersion,
  InterconnectCyclic_CustomData: InterconnectCyclic_CustomData,
  InterconnectCyclic_CustomData_tool_customData: InterconnectCyclic_CustomData_tool_customData,
  InterconnectCyclic_Command: InterconnectCyclic_Command,
  ArmLaterality: ArmLaterality,
  ModelId: ModelId,
  CompleteProductConfiguration: CompleteProductConfiguration,
  VisionModuleType: VisionModuleType,
  ProductConfigurationEndEffectorType: ProductConfigurationEndEffectorType,
  WristType: WristType,
  InterfaceModuleType: InterfaceModuleType,
  BrakeType: BrakeType,
  BaseType: BaseType,
  EndEffectorType: EndEffectorType,
  VisionNotification: VisionNotification,
  TranslationVector: TranslationVector,
  BitRate: BitRate,
  VisionConfig_ServiceVersion: VisionConfig_ServiceVersion,
  VisionEvent: VisionEvent,
  SensorFocusAction: SensorFocusAction,
  Sensor: Sensor,
  Resolution: Resolution,
  OptionIdentifier: OptionIdentifier,
  FrameRate: FrameRate,
  FocusAction: FocusAction,
  ExtrinsicParameters: ExtrinsicParameters,
  DistortionCoefficients: DistortionCoefficients,
  Option: Option,
  FocusPoint: FocusPoint,
  SensorFocusAction_action_parameters: SensorFocusAction_action_parameters,
  OptionValue: OptionValue,
  SensorIdentifier: SensorIdentifier,
  IntrinsicProfileIdentifier: IntrinsicProfileIdentifier,
  IntrinsicParameters: IntrinsicParameters,
  SensorSettings: SensorSettings,
  VisionConfig_RotationMatrix: VisionConfig_RotationMatrix,
  OptionInformation: OptionInformation,
  ManualFocus: ManualFocus,
  VisionConfig_RotationMatrixRow: VisionConfig_RotationMatrixRow,
  FollowCartesianTrajectoryGoal: FollowCartesianTrajectoryGoal,
  FollowCartesianTrajectoryFeedback: FollowCartesianTrajectoryFeedback,
  FollowCartesianTrajectoryAction: FollowCartesianTrajectoryAction,
  FollowCartesianTrajectoryResult: FollowCartesianTrajectoryResult,
  FollowCartesianTrajectoryActionResult: FollowCartesianTrajectoryActionResult,
  FollowCartesianTrajectoryActionFeedback: FollowCartesianTrajectoryActionFeedback,
  FollowCartesianTrajectoryActionGoal: FollowCartesianTrajectoryActionGoal,
};
