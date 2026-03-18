#include "JointController.hpp"

namespace Robot {
namespace Controllers {

JointController::JointController(std::shared_ptr<ServoDriver> servo_driver,
                                 std::shared_ptr<Config::ConfigManager> config_manager)
    : servo_driver_(servo_driver)
    , config_manager_(config_manager)
    , current_angles_{}
    , initialized_(false)
    , mapping_cache_{}
    , mapping_cached_(false)
{
    LOG_TAG_DEBUG("JOINT", "JointController created");
}

JointController::~JointController() {
    if (initialized_) {
        deinit();
    }
}

JointError JointController::init() {
    LOG_TAG_INFO("JOINT", "Initializing JointController");

    if (!servo_driver_) {
        LOG_TAG_ERROR("JOINT", "ServoDriver not provided");
        return JointError::SERVO_ERROR;
    }

    if (!config_manager_) {
        LOG_TAG_ERROR("JOINT", "ConfigManager not provided");
        return JointError::CONFIG_ERROR;
    }

    // 验证配置有效性
    if (!config_manager_->validateConfig()) {
        LOG_TAG_ERROR("JOINT", "Invalid joint configuration");
        return JointError::CONFIG_ERROR;
    }

    // 构建映射关系缓存
    if (!buildMappingCache()) {
        LOG_TAG_ERROR("JOINT", "Failed to build mapping cache");
        return JointError::CONFIG_ERROR;
    }

    // 初始化角度缓存为安全位置
    ThreeJointAngles safe_angles;
    safe_angles.hip_side = 0.0f;   // 髋关节侧摆中位
    safe_angles.hip_pitch = 0.0f;  // 髋关节俯仰中位
    safe_angles.knee_pitch = -90.0f; // 膝关节略微弯曲

    for (int i = 0; i < LEG_COUNT; i++) {
        current_angles_[i] = safe_angles;
    }

    initialized_ = true;
    LOG_TAG_INFO("JOINT", "JointController initialized successfully");
    return JointError::SUCCESS;
}

void JointController::deinit() {
    if (initialized_) {
        LOG_TAG_INFO("JOINT", "Deinitializing JointController");

        // 将所有关节移动到安全位置
        setSleepPose();

        initialized_ = false;
        LOG_TAG_DEBUG("JOINT", "JointController deinitialized");
    }
}

JointError JointController::setJointAngle(LegID leg_id, JointType joint_type, float angle) {
    // 验证参数
    if (!isValidLegId(leg_id)) {
        LOG_TAG_ERROR("JOINT", "Invalid leg ID: %d", static_cast<int>(leg_id));
        return JointError::INVALID_LEG_ID;
    }

    if (!isValidJointType(joint_type)) {
        LOG_TAG_ERROR("JOINT", "Invalid joint type: %d", static_cast<int>(joint_type));
        return JointError::INVALID_JOINT_TYPE;
    }

    if (!initialized_) {
        LOG_TAG_ERROR("JOINT", "JointController not initialized");
        return JointError::NOT_INITIALIZED;
    }

    // 验证并限制角度
    if (!isJointAngleValid(leg_id, joint_type, angle)) {
        float clamped_angle = clampJointAngle(leg_id, joint_type, angle);
        LOG_TAG_WARN("JOINT", "Joint angle %.2f° clamped to %.2f° for leg %d joint %d",
                     angle, clamped_angle, static_cast<int>(leg_id), static_cast<int>(joint_type));
        angle = clamped_angle;
    }

    // 转换为舵机角度
    float servo_angle;
    JointError result = jointToServoAngle(leg_id, joint_type, angle, servo_angle);
    if (result != JointError::SUCCESS) {
        LOG_TAG_ERROR("JOINT", "Failed to convert joint angle to servo angle");
        return result;
    }

    // 获取对应的舵机ID
    int servo_id = getServoId(leg_id, joint_type);

    // 硬件未就绪时，跳过下发，仅更新缓存
    if (!servo_driver_->isHardwareReady()) {
        // 更新缓存
        switch (joint_type) {
            case JointType::HIP_ROLL:
                current_angles_[static_cast<int>(leg_id)].hip_side = angle;
                break;
            case JointType::HIP_PITCH:
                current_angles_[static_cast<int>(leg_id)].hip_pitch = angle;
                break;
            case JointType::KNEE_PITCH:
                current_angles_[static_cast<int>(leg_id)].knee_pitch = angle;
                break;
        }
        LOG_TAG_DEBUG("JOINT", "No hardware, skip setAngle for leg %d joint %d (%.2f°, servo %.2f°)",
                     static_cast<int>(leg_id), static_cast<int>(joint_type), angle, servo_angle);
        return JointError::SUCCESS;
    }

    // 发送到舵机驱动器
    ServoError servo_result = servo_driver_->setAngle(servo_id, servo_angle);
    if (servo_result != ServoError::SUCCESS) {
        LOG_TAG_WARN("JOINT", "Servo setAngle failed: %s (leg %d joint %d)",
                     ServoDriver::getErrorString(servo_result),
                     static_cast<int>(leg_id), static_cast<int>(joint_type));
        // 不中断流程，视为仿真成功
        return JointError::SUCCESS;
    }

    // 更新缓存
    switch (joint_type) {
        case JointType::HIP_ROLL:
            current_angles_[static_cast<int>(leg_id)].hip_side = angle;
            break;
        case JointType::HIP_PITCH:
            current_angles_[static_cast<int>(leg_id)].hip_pitch = angle;
            break;
        case JointType::KNEE_PITCH:
            current_angles_[static_cast<int>(leg_id)].knee_pitch = angle;
            break;
    }

    LOG_TAG_DEBUG("JOINT", "Set leg %d joint %d to %.2f° (servo %d: %.2f°)",
                  static_cast<int>(leg_id), static_cast<int>(joint_type),
                  angle, servo_id, servo_angle);

    return JointError::SUCCESS;
}

JointError JointController::getJointAngle(LegID leg_id, JointType joint_type, float& angle) const {
    if (!isValidLegId(leg_id)) {
        return JointError::INVALID_LEG_ID;
    }

    if (!isValidJointType(joint_type)) {
        return JointError::INVALID_JOINT_TYPE;
    }

    if (!initialized_) {
        return JointError::NOT_INITIALIZED;
    }

    const auto& leg_angles = current_angles_[static_cast<int>(leg_id)];
    switch (joint_type) {
        case JointType::HIP_ROLL:
            angle = leg_angles.hip_side;
            break;
        case JointType::HIP_PITCH:
            angle = leg_angles.hip_pitch;
            break;
        case JointType::KNEE_PITCH:
            angle = leg_angles.knee_pitch;
            break;
    }

    return JointError::SUCCESS;
}

JointError JointController::setLegAngles(LegID leg_id, const ThreeJointAngles& angles) {
    if (!isValidLegId(leg_id)) {
        return JointError::INVALID_LEG_ID;
    }

    LOG_TAG_DEBUG("JOINT", "Setting leg %d angles: hip_side=%.2f, hip_pitch=%.2f, knee_pitch=%.2f",
                  static_cast<int>(leg_id), angles.hip_side, angles.hip_pitch, angles.knee_pitch);

    // 设置各个关节
    JointError result;

    result = setJointAngle(leg_id, JointType::HIP_ROLL, angles.hip_side);
    if (result != JointError::SUCCESS) return result;

    result = setJointAngle(leg_id, JointType::HIP_PITCH, angles.hip_pitch);
    if (result != JointError::SUCCESS) return result;

    result = setJointAngle(leg_id, JointType::KNEE_PITCH, angles.knee_pitch);
    if (result != JointError::SUCCESS) return result;

    LOG_TAG_DEBUG("JOINT", "Successfully set all angles for leg %d", static_cast<int>(leg_id));
    return JointError::SUCCESS;
}

JointError JointController::getLegAngles(LegID leg_id, ThreeJointAngles& angles) const {
    if (!isValidLegId(leg_id)) {
        return JointError::INVALID_LEG_ID;
    }

    if (!initialized_) {
        return JointError::NOT_INITIALIZED;
    }

    angles = current_angles_[static_cast<int>(leg_id)];
    return JointError::SUCCESS;
}

JointError JointController::setAllJointAngles(const AllLegJoints& all_angles) {
    LOG_TAG_DEBUG("JOINT", "Setting angles for all legs");

    for (int i = 0; i < LEG_COUNT; i++) {
        LegID leg_id = static_cast<LegID>(i);

        // 从AllLegJoints中提取ThreeJointAngles
        ThreeJointAngles leg_angles;
        leg_angles.hip_side = all_angles[i][0].angle;    // HIP_ROLL
        leg_angles.hip_pitch = all_angles[i][1].angle;   // HIP_PITCH
        leg_angles.knee_pitch = all_angles[i][2].angle;  // KNEE_PITCH

        JointError result = setLegAngles(leg_id, leg_angles);
        if (result != JointError::SUCCESS) {
            LOG_TAG_ERROR("JOINT", "Failed to set angles for leg %d", i);
            return result;
        }
    }

    LOG_TAG_DEBUG("JOINT", "Successfully set angles for all legs");
    return JointError::SUCCESS;
}

JointError JointController::getAllJointAngles(AllLegJoints& all_angles) const {
    if (!initialized_) {
        return JointError::NOT_INITIALIZED;
    }

    for (int i = 0; i < LEG_COUNT; i++) {
        all_angles[i][0] = JointAngle(current_angles_[i].hip_side);
        all_angles[i][1] = JointAngle(current_angles_[i].hip_pitch);
        all_angles[i][2] = JointAngle(current_angles_[i].knee_pitch);
    }

    return JointError::SUCCESS;
}

JointError JointController::jointToServoAngle(LegID leg_id, JointType joint_type,
                                              float joint_angle, float& servo_angle) const {
    if (!isValidLegId(leg_id) || !isValidJointType(joint_type)) {
        return JointError::INVALID_LEG_ID;
    }

    const auto& mapping = getJointMapping(leg_id, joint_type);
    servo_angle = applyJointMapping(mapping, joint_angle);

    LOG_TAG_DEBUG("JOINT", "Joint->Servo: leg %d joint %d: %.2f° -> %.2f°",
                  static_cast<int>(leg_id), static_cast<int>(joint_type),
                  joint_angle, servo_angle);

    return JointError::SUCCESS;
}

JointError JointController::servoToJointAngle(LegID leg_id, JointType joint_type,
                                              float servo_angle, float& joint_angle) const {
    if (!isValidLegId(leg_id) || !isValidJointType(joint_type)) {
        return JointError::INVALID_LEG_ID;
    }

    const auto& mapping = getJointMapping(leg_id, joint_type);
    joint_angle = reverseJointMapping(mapping, servo_angle);

    LOG_TAG_DEBUG("JOINT", "Servo->Joint: leg %d joint %d: %.2f° -> %.2f°",
                  static_cast<int>(leg_id), static_cast<int>(joint_type),
                  servo_angle, joint_angle);

    return JointError::SUCCESS;
}

JointError JointController::getJointLimits(LegID leg_id, JointType joint_type,
                                           float& min_angle, float& max_angle) const {
    if (!isValidLegId(leg_id) || !isValidJointType(joint_type)) {
        return JointError::INVALID_LEG_ID;
    }

    const auto& mapping = getJointMapping(leg_id, joint_type);
    min_angle = mapping.joint_min;
    max_angle = mapping.joint_max;

    return JointError::SUCCESS;
}

bool JointController::isJointAngleValid(LegID leg_id, JointType joint_type, float angle) const {
    float min_angle, max_angle;
    if (getJointLimits(leg_id, joint_type, min_angle, max_angle) != JointError::SUCCESS) {
        return false;
    }

    return angle >= min_angle && angle <= max_angle;
}

float JointController::clampJointAngle(LegID leg_id, JointType joint_type, float angle) const {
    float min_angle, max_angle;
    if (getJointLimits(leg_id, joint_type, min_angle, max_angle) != JointError::SUCCESS) {
        return angle;
    }

    return std::max(min_angle, std::min(max_angle, angle));
}

int JointController::getServoId(LegID leg_id, JointType joint_type) const {
    // 舵机ID映射: 每条腿3个舵机，按顺序排列
    // 左前腿(0): 舵机0,1,2  右前腿(1): 舵机3,4,5
    // 左后腿(2): 舵机6,7,8  右后腿(3): 舵机9,10,11
    int leg_index = static_cast<int>(leg_id);
    int joint_index = static_cast<int>(joint_type);
    return leg_index * JOINTS_PER_LEG + joint_index;
}

JointError JointController::setSleepPose() {
    LOG_TAG_INFO("JOINT", "Setting sleep pose (lying down)");

    AllLegJoints sleep_pose;

    // 趴下姿势 - 所有腿部放平
    for (int i = 0; i < LEG_COUNT; i++) {
        sleep_pose[i][0] = JointAngle(0.0f);     // HIP_ROLL 中位
        sleep_pose[i][1] = JointAngle(60.0f);    // HIP_PITCH 向前
        sleep_pose[i][2] = JointAngle(-120.0f);  // KNEE_PITCH 弯曲
    }

    return setAllJointAngles(sleep_pose);
}

JointError JointController::setStandPose() {
    LOG_TAG_INFO("JOINT", "Setting stand pose");

    AllLegJoints stand_pose;

    // 站立姿势
    for (int i = 0; i < LEG_COUNT; i++) {
        stand_pose[i][0] = JointAngle(0.0f);     // HIP_ROLL 中位
        stand_pose[i][1] = JointAngle(30.0f);    // HIP_PITCH 略微向后
        stand_pose[i][2] = JointAngle(-60.0f);   // KNEE_PITCH 适度弯曲
    }

    return setAllJointAngles(stand_pose);
}

const char* JointController::getErrorString(JointError error) {
    switch (error) {
        case JointError::SUCCESS:           return "Success";
        case JointError::INVALID_LEG_ID:    return "Invalid leg ID";
        case JointError::INVALID_JOINT_TYPE:return "Invalid joint type";
        case JointError::INVALID_ANGLE:     return "Invalid angle";
        case JointError::SERVO_ERROR:       return "Servo error";
        case JointError::NOT_INITIALIZED:   return "Not initialized";
        case JointError::CONFIG_ERROR:      return "Configuration error";
        default:                            return "Unknown error";
    }
}

const Config::LegMappingConfig::JointMapping&
JointController::getJointMapping(LegID leg_id, JointType joint_type) const {
    return config_manager_->getMappingConfig().getJointMapping(leg_id, joint_type);
}

float JointController::applyJointMapping(const Config::LegMappingConfig::JointMapping& mapping,
                                         float joint_angle) const {
    // 应用线性映射: servo_angle = k * joint_angle + b
    return mapping.k * joint_angle + mapping.b;
}

float JointController::reverseJointMapping(const Config::LegMappingConfig::JointMapping& mapping,
                                           float servo_angle) const {
    // 反向映射: joint_angle = (servo_angle - b) / k
    if (mapping.k == 0.0f) {
        LOG_TAG_ERROR("JOINT", "Invalid mapping coefficient k=0");
        return servo_angle;  // 避免除零
    }
    return (servo_angle - mapping.b) / mapping.k;
}

bool JointController::buildMappingCache() {
    if (!config_manager_) {
        LOG_TAG_WARN("JOINT", "ConfigManager is null, using default mappings");
        // 使用默认映射填充缓存
        for (int leg = 0; leg < LEG_COUNT; leg++) {
            for (int joint = 0; joint < JOINTS_PER_LEG; joint++) {
                mapping_cache_[leg][joint] = CachedJointMapping(1.0f, 90.0f, true);
            }
        }
        mapping_cached_ = true;
        return true;
    }

    LOG_TAG_INFO("JOINT", "Building joint mapping cache from ConfigManager...");

    // 从ConfigManager一次性获取所有映射关系
    const auto& mapping_config = config_manager_->getMappingConfig();

    for (int leg = 0; leg < LEG_COUNT; leg++) {
        for (int joint = 0; joint < JOINTS_PER_LEG; joint++) {
            LegID leg_id = static_cast<LegID>(leg);
            JointType joint_type = static_cast<JointType>(joint);

            const auto& joint_mapping = mapping_config.getJointMapping(leg_id, joint_type);

            // 缓存映射参数
            mapping_cache_[leg][joint] = CachedJointMapping(
                joint_mapping.k,
                joint_mapping.b,
                joint_mapping.enabled
            );

            LOG_TAG_DEBUG("JOINT", "Cached mapping L%dJ%d: k=%.2f, b=%.2f, enabled=%s",
                         leg, joint, joint_mapping.k, joint_mapping.b,
                         joint_mapping.enabled ? "YES" : "NO");
        }
    }

    mapping_cached_ = true;
    LOG_TAG_INFO("JOINT", "Mapping cache built successfully for %d legs x %d joints",
                 LEG_COUNT, JOINTS_PER_LEG);
    return true;
}

const CachedJointMapping& JointController::getCachedMapping(LegID leg_id, JointType joint_type) const {
    if (!mapping_cached_) {
        LOG_TAG_ERROR("JOINT", "Mapping cache not built, returning default");
        static const CachedJointMapping default_mapping(1.0f, 90.0f, true);
        return default_mapping;
    }

    int leg_index = static_cast<int>(leg_id);
    int joint_index = static_cast<int>(joint_type);

    if (leg_index < 0 || leg_index >= LEG_COUNT ||
        joint_index < 0 || joint_index >= JOINTS_PER_LEG) {
        LOG_TAG_ERROR("JOINT", "Invalid indices: leg %d, joint %d", leg_index, joint_index);
        static const CachedJointMapping default_mapping(1.0f, 90.0f, true);
        return default_mapping;
    }

    return mapping_cache_[leg_index][joint_index];
}

JointError JointController::jointToServoAngleFast(LegID leg_id, JointType joint_type,
                                                  float joint_angle, float& servo_angle) const {
    if (!isValidLegId(leg_id) || !isValidJointType(joint_type)) {
        return JointError::INVALID_LEG_ID;
    }

    if (!mapping_cached_) {
        LOG_TAG_WARN("JOINT", "Cache not available, falling back to standard method");
        return jointToServoAngle(leg_id, joint_type, joint_angle, servo_angle);
    }

    // 高性能：直接访问缓存，避免频繁查询ConfigManager
    const CachedJointMapping& mapping = getCachedMapping(leg_id, joint_type);

    // 应用缓存的线性映射: servo_angle = k * joint_angle + b
    servo_angle = mapping.jointToServo(joint_angle);

    // 确保在舵机范围内 (0-180度)
    servo_angle = std::max(0.0f, std::min(180.0f, servo_angle));

    LOG_TAG_DEBUG("JOINT", "Joint->Servo (cached): L%dJ%d: %.2f° -> %.2f° (k=%.2f, b=%.2f)",
                  static_cast<int>(leg_id), static_cast<int>(joint_type),
                  joint_angle, servo_angle, mapping.k, mapping.b);

    return JointError::SUCCESS;
}

JointError JointController::servoToJointAngleFast(LegID leg_id, JointType joint_type,
                                                  float servo_angle, float& joint_angle) const {
    if (!isValidLegId(leg_id) || !isValidJointType(joint_type)) {
        return JointError::INVALID_LEG_ID;
    }

    if (!mapping_cached_) {
        LOG_TAG_WARN("JOINT", "Cache not available, falling back to standard method");
        return servoToJointAngle(leg_id, joint_type, servo_angle, joint_angle);
    }

    // 高性能：直接访问缓存，避免频繁查询ConfigManager
    const CachedJointMapping& mapping = getCachedMapping(leg_id, joint_type);

    // 反向映射: joint_angle = (servo_angle - b) / k
    joint_angle = mapping.servoToJoint(servo_angle);

    LOG_TAG_DEBUG("JOINT", "Servo->Joint (cached): L%dJ%d: %.2f° -> %.2f° (k=%.2f, b=%.2f)",
                  static_cast<int>(leg_id), static_cast<int>(joint_type),
                  servo_angle, joint_angle, mapping.k, mapping.b);

    return JointError::SUCCESS;
}

} // namespace Controllers
} // namespace Robot