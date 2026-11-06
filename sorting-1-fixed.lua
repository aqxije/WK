-- =============================================================================
-- 版本：v3.4.9 直线加速度修正 | v3.4.8 节拍优化(关节速度+拍照过渡) | v3.4.7 拍照Z产品偏移 | v3.4.6 方向缺失防呆 | v3.4.5 reverseOrder参数修正 | v3.4.4 产品参数框架版 | v3.4.3 相机读超时防护与换层预确认版 | v3.4.2 安全检查与相机空返回修复版 | v3.4.1 动态速度倍率(42001)修复版 | v3.4.0 取料前测电压版
-- 用途：FD5-SORTING 上位机Modbus通讯 + 机械臂取放料动作流程
-- 优化项：取料流程中新增测电压步骤（料取位Z方向下移固定距离测量，测完回到抓取位再抓取）
--         新增 ENABLE_VOLTAGE_CHECK 启用开关 | VOLTAGE_CHECK_OFFSET 测电压Z偏移
--         新增 REG_VOLTAGE_DONE 寄存器 | ERR_VOLTAGE_TIMEOUT 错误码
-- =============================================================================

-- ===================== 第一部分：全局参数配置 =====================
-- 运动参数：关节运动单位为°/s、°/s2；直线运动单位为mm/s、mm/s2
SPEED_J = 110         -- 关节运动速度（v3.4.8: 90→110试验值，现场可阶梯上调至130，注意末端振动与拍照清晰度）
ACC_J   = 200         -- 关节加速度（v3.4.8: 160→200）
SPEED_L = 1500        -- 直线运动速度
ACC_L   = 2000        -- 直线加速度（v3.4.9修正：官方上限2000，原值超限会被钳制）
setMoveRate(30)       -- 设置初始运动速率比例
setTool(2)            -- 设置工具坐标系
wait(2000)            -- 等待系统初始化

-- 工业相机通讯参数
CAMERA_IP      = "192.168.3.102"  -- 相机IP地址
CAMERA_PORT    = 50000            -- 相机端口号
SOCKET_TIMEOUT = 30000            -- 通讯超时时间(ms)
RETRY_MAX      = 3                -- 最大重试次数

-- 高度参数（单位：mm）
SCAN_HEIGHT       = 350   -- 扫码位预留高度
SAFE_HEIGHT       = 300   -- 运动安全高度
PICK_HEIGHT_OFFSET  = 46   -- 取料高度补偿
PLACE_HEIGHT_OFFSET = 46   -- 放料高度补偿

-- 寻位功能参数
SEARCH_IO_NUM     = 10    -- 寻位检测DI地址
SEARCH_LIFT_HEIGHT = 100  -- 寻位触发后Z轴抬升高度(mm)

-- 坐标偏差校验参数
POSE_DEV_XYZ_THRESHOLD = 50.0    -- XYZ单轴位置允许偏差(mm)
POSE_DEV_R_THRESHOLD   = 361     -- RX/RY/RZ单轴旋转允许偏差(°)

-- 电芯矩阵步长参数（单位：mm）
PICK_STEP_X  = 66     -- 取料X方向步长
PICK_STEP_Y  = 245    -- 取料Y方向步长
PICK_STEP_Z  = 260    -- 取料Z方向层间步长
PLACE_STEP_X = 66     -- 放料X方向步长
PLACE_STEP_Y = 245    -- 放料Y方向步长
PLACE_STEP_Z = 260    -- 放料Z方向层间步长

-- 第4层斜拍专属步长参数（Z方向为0，表示第4层Z不偏移）
PICK_STEP_X_L4  = 66    -- 第4层取料X方向步长
PICK_STEP_Y_L4  = 245   -- 第4层取料Y方向步长
PICK_STEP_Z_L4  = 0     -- 第4层取料Z方向步长（不偏移）
PLACE_STEP_X_L4 = 66    -- 第4层放料X方向步长
PLACE_STEP_Y_L4 = 245   -- 第4层放料Y方向步长
PLACE_STEP_Z_L4 = 0     -- 第4层放料Z方向步长（不偏移）

-- 误差偏移调整参数
PICK_X_OFFSET  = 0     -- 取料X方向误差补偿
PICK_Y_OFFSET  = 0     -- 取料Y方向误差补偿
PLACE_X_OFFSET = 0     -- 放料X方向误差补偿
PLACE_Y_OFFSET = 0     -- 放料Y方向误差补偿

-- 托盘与料垛规格
-- 注意：托盘规格唯一事实源为产品参数表（loadProduct同步写REG_MAX_*与以下常量），上位机请勿直接改写REG_MAX_*
PALLET_ROWS  = 4       -- 托盘行数
PALLET_COLS  = 15      -- 托盘列数
PALLET_TOTAL = PALLET_ROWS * PALLET_COLS  -- 托盘总容量
MAX_LAYER    = 4       -- 最大层数
MAX_STATION  = 10      -- 最大支持站点数量

-- 放料顺序配置
PLACE_REVERSE_ORDER = true  -- true=倒序放置 false=正序放置

-- 电芯检测IO地址
CELL_DETECTION_IO   = 8    -- 夹爪电芯检测DI（高电平=有料）
CELL_DETECTION_IO_1 = 1    -- 放料位空穴检测DI（高电平=有料）

-- 系统轮询间隔
POLL_INTERVAL = 100  -- 单位：ms

-- 柔性触发切换
SEARCH_CHANGE = true  -- true=启用寻位指令 false=禁用寻位指令

-- 放料管理模式切换
-- false=站点自管理放料（原有逻辑）  true=上位机计数放料（上位机下发计数+层号+站点）
-- 运行时可由上位机信号 COIL_CONFIRM_2(9034) 动态覆盖（1=上位机计数放料）
USE_HOST_PLACE_MODEL = true

-- 横纵固定值校验切换
MATRIX_CHANGE = true  -- true=启用 false=禁用

-- 测电压功能切换
ENABLE_VOLTAGE_CHECK = true  -- true=启用取料前测电压 false=禁用取料前测电压
VOLTAGE_CHECK_OFFSET = -6     -- 测电压位相对取料位的Z方向偏移(mm)

-- ===================== 第二部分：Modbus寄存器地址定义 =====================
-- 上位机下发参数（可读写）
REG_ROW           = 49100  -- 取料行号
REG_COL           = 49102  -- 取料列号
REG_LAYER         = 49104  -- 取料层号
REG_STATION       = 49106  -- 目标放料站点号
REG_CMD_CODE      = 49108  -- 指令码（1=取料，2=放料）
REG_SCAN_DONE     = 49110  -- 扫码完成标志（上位机置1）
REG_VOLTAGE_DONE  = 49110  -- 测电压完成标志（上位机置2）
REG_MAX_ROW       = 49112  -- 当前托盘最大行数
REG_MAX_COL       = 49114  -- 当前托盘最大列数
REG_MAX_LAYER     = 49116  -- 当前料垛最大层数
-- 行列表由loadProduct按产品参数表写入，全脚本行列计算口径已统一以此寄存器为准（getTrayDims）
REG_STATION_SIDE  = 49118  -- 站点侧位标志（0=左侧，1=右侧）
REG_PICK_MATRIX   = 49120  -- 取料排列方向
REG_PLACE_MATRIX  = 49130  -- 放料排列方向

-- 人工交互寄存器
REG_CONFIRM_REMOVE = 49122  -- 来料垛移除确认（预留：本版本未使用）
REG_CONFIRM_FULL   = 49124  -- 放料换层确认
REG_CONFIRM_RESET  = 49126  -- 放料清垛确认
REG_CMD_ACK        = 49128  -- 指令应答（0=待处理/1=执行中/2=完成/负数=错误码）

-- 扩展交互寄存器（可读写）
REG_PLACE_ROW   = 50000  -- 上位机放料行号
REG_PLACE_COL   = 50002  -- 上位机放料列号
REG_PLACE_LAYER = 50004  -- 上位机放料层号
REG_MOVE_RATE   = 42001  -- 库卓系统速度倍率寄存器（上位机下发0~100），负载切换线程消费
REG_ERROR_CODE  = 50008  -- 错误码寄存器
REG_STACK_STATION  = 50010  -- 上位机清层、清垛站号
REG_PLACE_COUNT = 50012  -- 上位机放料计数号
REG_ROBOT_STAT  = 50014  -- 机器人整体状态（0=空闲，1=动作中，3=错误）
REG_VOLTAGE_START  = 50016  -- 上位机确认需要测电压信号
REG_PRODUCT_ID  = 50018  -- 产品型号号（上位机写入，指令间隙切换并加载产品参数）

-- 机器人状态反馈（只读）
REG_PICK_STAT   = 49002  -- 取料状态（0=未完成，1=完成）
REG_PLACE_STAT  = 49004  -- 放料状态（0=未完成，1=完成）
REG_CUR_ROW     = 49006  -- 当前取料行号
REG_CUR_COL     = 49008  -- 当前取料列号
REG_CUR_LAYER   = 49010  -- 当前取料层号
REG_ARM_STAT    = 49012  -- 机械臂子状态（0=空闲/1=取料中/2=放料中/3=错误/4=已到扫码位/5=已抓取/6=测电压中）

-- 布尔量寄存器（只读）
COIL_GRIPPER_OPEN   = 9000  -- 夹爪张开反馈
COIL_GRIPPER_CLOSE  = 9001  -- 夹爪闭合反馈

-- 布尔量寄存器（读写）
COIL_GRIPPER_HOME   = 9032  -- 夹爪回零确认
COIL_CONFIRM_1      = 9033  -- 电芯排列方向确认（预留：本版本未使用）
COIL_CONFIRM_2      = 9034  -- 使用上位机放料位确认

-- 夹爪硬件IO地址
GRIPPER_CLOSE_OUT = 0    -- 夹爪闭合输出
GRIPPER_OPEN_OUT  = 1    -- 夹爪张开输出
GRIPPER_OPEN_IN   = 14   -- 夹爪张开到位输入
GRIPPER_CLOSE_IN  = 15   -- 夹爪闭合到位输入

-- ===================== 第二A部分：产品参数表与切换 =====================
-- 产品号由上位机写入 REG_PRODUCT_ID(50018)。切换仅在主循环指令间隙生效：
-- 动作进行中变号顺延到当前指令完成后加载；未定义产品号将报1011暂停。
-- 新增产品：在 PRODUCTS 表中加一项，并完成点位示教/相机工程/高度区间标定。
PRODUCTS = {
    [1] = {
        name        = "标准款 4x15x4",
        rows        = 4,                        -- 托盘行数
        cols        = 15,                       -- 托盘列数
        layers      = 4,                        -- 料垛最大层数
        pickStep    = {66, 245, 260},           -- 取料X/Y/Z步长(mm)
        placeStep   = {66, 245, 260},           -- 放料X/Y/Z步长(mm)
        pickStepL4  = {66, 245, 0},             -- 第4层取料X/Y/Z步长
        placeStepL4 = {66, 245, 0},             -- 第4层放料X/Y/Z步长
        pickHeightOffset  = 46,                 -- 取料高度补偿(mm)
        placeHeightOffset = 46,                 -- 放料高度补偿(mm)
        layerHeights = {                        -- 方向判定高度区间表(层号={下限,上限})
            {-450, -420}, {-185, -155}, {80, 110}, {345, 375},
        },
        camId = { grab = 1, grabL4 = 2, dir = 3, place = 4, placeL4 = 5 },  -- 相机工程ID
        camZShift = 0,                       -- 拍照基准Z平移(mm)：吸收产品层高累计差，无需重拍示教点
    },
}

CUR_PRODUCT    = nil   -- 当前产品参数表（loadProduct成功后生效）
CUR_PRODUCT_ID = nil   -- 当前已加载产品号

--[[
函数名：loadProduct
功能：加载指定产品参数：校验参数表→覆盖矩阵常量→同步行列表寄存器→记录当前产品
      运行中切换产品时，旧产品的站点记忆与取放料基准全部作废
参数：id - 产品号
返回：bool true=加载成功 false=产品未定义或参数不合法
]]
function loadProduct(id)
    local p = PRODUCTS[id]
    if not p then
        local ids = {}
        for k in pairs(PRODUCTS) do ids[#ids + 1] = tostring(k) end
        table.sort(ids)
        print(string.format("[产品切换] 产品号%d未定义，已定义产品: %s", id, table.concat(ids, ",")))
        return false
    end
    if type(p.rows) ~= "number" or p.rows < 1 or p.rows ~= math.floor(p.rows)
        or type(p.cols) ~= "number" or p.cols < 1 or p.cols ~= math.floor(p.cols)
        or type(p.layers) ~= "number" or p.layers < 1 or p.layers ~= math.floor(p.layers)
        or type(p.layerHeights) ~= "table" or #p.layerHeights ~= p.layers
        or type(p.camId) ~= "table"
        or type(p.camZShift) ~= "number" then
        print(string.format("[产品切换] 产品%d(%s)参数表不合法，不予加载", id, tostring(p.name)))
        return false
    end

    -- 运行中切换产品：旧产品的站点记忆与基准作废（原地清空，保持断电保持变量绑定）
    if CUR_PRODUCT_ID ~= nil and CUR_PRODUCT_ID ~= id then
        if type(g_stationState) == "table" then
            for k in pairs(g_stationState) do g_stationState[k] = nil end
        end
        if type(g_placeBasePose) == "table" then
            for k in pairs(g_placeBasePose) do g_placeBasePose[k] = nil end
        end
        if type(g_pickBasePose) == "table" then
            for k in pairs(g_pickBasePose) do g_pickBasePose[k] = nil end
        end
        g_lastPickLayer = nil
        g_lastPickMatrix = nil
        g_pickMatrixForLayer = nil
        print("[产品切换] 产品变更，已清空站点状态与取放料基准")
    end

    -- 覆盖矩阵几何常量（与第一部分定义的同名全局同步）
    PALLET_ROWS  = p.rows
    PALLET_COLS  = p.cols
    PALLET_TOTAL = p.rows * p.cols
    MAX_LAYER    = p.layers
    PICK_STEP_X, PICK_STEP_Y, PICK_STEP_Z = p.pickStep[1], p.pickStep[2], p.pickStep[3]
    PLACE_STEP_X, PLACE_STEP_Y, PLACE_STEP_Z = p.placeStep[1], p.placeStep[2], p.placeStep[3]
    PICK_STEP_X_L4, PICK_STEP_Y_L4, PICK_STEP_Z_L4 = p.pickStepL4[1], p.pickStepL4[2], p.pickStepL4[3]
    PLACE_STEP_X_L4, PLACE_STEP_Y_L4, PLACE_STEP_Z_L4 = p.placeStepL4[1], p.placeStepL4[2], p.placeStepL4[3]
    PICK_HEIGHT_OFFSET  = p.pickHeightOffset
    PLACE_HEIGHT_OFFSET = p.placeHeightOffset

    -- 行列表寄存器为唯一事实源，与常量同步写入
    setRegisterInt(REG_MAX_ROW, p.rows)
    setRegisterInt(REG_MAX_COL, p.cols)
    setRegisterInt(REG_MAX_LAYER, p.layers)

    CUR_PRODUCT    = p
    CUR_PRODUCT_ID = id
    print(string.format("[产品切换] 已加载产品%d: %s（%d行%d列%d层，拍照Z偏移%dmm）", id, tostring(p.name), p.rows, p.cols, p.layers, p.camZShift))
    return true
end

--[[
函数名：getTrayDims
功能：读取当前托盘行列层规格（单一事实源：REG_MAX_*寄存器，非法值回退常量）
返回：maxRow, maxCol, maxLayer, total
]]
function getTrayDims()
    local maxRow   = getRegisterInt(REG_MAX_ROW)
    local maxCol   = getRegisterInt(REG_MAX_COL)
    local maxLayer = getRegisterInt(REG_MAX_LAYER)
    if type(maxRow) ~= "number" or maxRow <= 0 then maxRow = PALLET_ROWS end
    if type(maxCol) ~= "number" or maxCol <= 0 then maxCol = PALLET_COLS end
    if type(maxLayer) ~= "number" or maxLayer <= 0 then maxLayer = MAX_LAYER end
    return maxRow, maxCol, maxLayer, maxRow * maxCol
end

--[[
函数名：zShiftPose
功能：返回点位副本并按当前产品camZShift平移Z（不改动示教点位本体，无持久化污染）
参数：pose - 示教点位
返回：平移后的点位副本
]]
function zShiftPose(p)
    local dz = CUR_PRODUCT and CUR_PRODUCT.camZShift or 0
    return {cp = {p.cp[1], p.cp[2], p.cp[3] + dz, p.cp[4], p.cp[5], p.cp[6]}}
end

-- ===================== 第三部分：全局状态变量 =====================
-- 【断电保持变量】（控制器持久化存储，此处不做初始化）
-- g_cellGrabbed       电芯抓取标志
-- g_targetStation     当前任务目标站点（预留：本版本未使用）
-- g_row               当前取料行号
-- g_col               当前取料列号
-- g_layer             当前取料层号
-- g_cmdCode           当前指令码
-- g_cellMatrix        电芯排列方向 0=X优先 1=Y优先
-- g_stationState      多站点状态表
-- g_lastPickMatrix    上一次取料排列方向
-- g_lastPickLayer     上一次取料层号
-- g_lastPlaceMatrix   上一次放料排列方向（预留：本版本未使用，放料一致性由g_stationState承担）
-- g_lastPlaceLayer    上一次放料层号（预留：本版本未使用，放料一致性由g_stationState承担）
-- g_pickMatrixForLayer  当前层取料方向
-- g_placeMatrixForLayer 当前层放料方向（预留：本版本未使用）
-- USE_CAMERA_MATRIX_DETECTION  启用相机方向判定
-- ERROR_OFFSET_SELECT 启用误差矩阵偏移
-- ENABLE_CELL_DETECTION 启用电芯IO检测

-- 【非断电保持变量】
g_cameraConnected = false   -- 相机连接状态标志
g_PickRow   = 0             -- 当前取料行号
g_PickCol   = 0             -- 当前取料列号
g_PlaceRow  = 0             -- 当前放料行号
g_PlaceCol  = 0             -- 当前放料列号
g_cellPresent = nil         -- 电芯存在状态（nil=未知 false=无料 true=有料）

-- 坐标基准记录表
-- g_pickBasePose  = {}  -- key=层号，value=基准位姿
-- g_placeBasePose = {}  -- key=站点号_层号，value=基准位姿 【站点+层复合key，每层独立基准】

-- ===================== 第四部分：点位说明 =====================
-- 取放1~3层基准拍照点位：
-- 取料侧：pCamera_Grab_XZ / pCamera_Grab_XY / pCamera_Grab_YZ / pCamera_Grab_YY
-- 放料侧：pCamera_Place_XZ / pCamera_Place_XY / pCamera_Place_YZ / pCamera_Place_YY
-- 第4层斜拍专属点位：后缀加 _L4
-- 关节过渡点：GD_ZC（左侧） / GD_YC（右侧） / pHome（回程过渡点）
-- 工艺TCP点：pStation_Grab / pStation_Grab_Up / pStation_Grab_Scan
--           pStation_Place / pStation_Place_Up

-- ===================== 第五部分：错误码定义与工具函数 =====================
-- ============================================================
-- 错误码分类方案：
--   1xxx  系统错误     （初始化、回原、信号、AGV等系统级故障）
--   2xxx  夹爪控制错误 （夹爪张开/闭合/回零动作超时或失败）
--   3xxx  相机通讯错误 （相机连接/触发/读取/坐标解析异常）
--   4xxx  电芯状态错误 （电芯存在性、抓取记忆一致性、层数判定等）
--   5xxx  参数/防呆错误（参数合法性、矩阵边界、方向一致性、坐标偏差等）
-- 错误码细分原则：涉及取料/放料双环节的错误码拆分为 _PICK/_PLACE；
--   涉及初始化/主循环的拆分为 _INIT/_RUN，便于上位机按环节定位排查
-- ============================================================

-- 错误类型定义
-- ERR_TYPE_PAUSE = 1  -- 暂停类：可人工恢复后继续
-- ERR_TYPE_STOP  = 2  -- 停止类：严重故障，需复位

-- ============ 1xxx 系统错误 ============
ERR_SYS_NONE            = 0     -- 无错误
ERR_SYS_SEARCH_TRIGGER  = 1001  -- 寻位安全信号触发，运动中止
ERR_SYS_ARM_NOT_HOME    = 1002  -- 机械臂不在原点且无法自动回原
ERR_SYS_SCAN_TIMEOUT    = 1003  -- 扫码等待超时
ERR_SYS_PALLET_TIMEOUT  = 1004  -- 换盘等待超时
ERR_SYS_STACK_TIMEOUT   = 1005  -- 换垛等待超时
ERR_VOLTAGE_TIMEOUT     = 1006  -- 测电压等待超时
ERR_SCAN_RETRY_FAIL     = 1007  -- 扫码等待失败（重试完）
ERR_VOLTAGE_RETRY_FAIL  = 1008  -- 测电压等待失败（重试完）
ERR_VOLTAGE_SIGN_TIMEOUT  = 1009  -- 是否需要测电压信号等待超时
ERR_SPEED_RATE_INVALID   = 1010  -- 速度倍率无效（42001为0或超范围）
ERR_PRODUCT_UNDEFINED   = 1011  -- 产品号未定义或产品参数表不合法

-- ============ 2xxx 夹爪控制错误 ============
ERR_GRIPPER_OPEN_TIMEOUT    = 2001  -- 夹爪张开超时
ERR_GRIPPER_CLOSE_TIMEOUT   = 2002  -- 夹爪闭合超时
ERR_GRIPPER_HOME_TIMEOUT    = 2003  -- 夹爪回零超时
ERR_GRIPPER_NOT_OPEN        = 2004  -- 夹爪不在张开状态

-- ============ 3xxx 相机通讯错误 ============
ERR_CAM_CONNECT_FAIL        = 3001  -- 相机连接失败（重试完）
ERR_CAM_TRIG_SEND_FAIL      = 3002  -- 触发指令发送失败
ERR_CAM_TRIG_NO_RESP        = 3003  -- 未接收触发响应
ERR_CAM_TRIG_CODE_ERR       = 3004  -- 触发响应指令码异常
ERR_CAM_READ_SEND_FAIL      = 3005  -- 读取指令发送失败
ERR_CAM_RESULT_NO_RESP      = 3006  -- 未接收结果响应
ERR_CAM_RESULT_CODE_ERR     = 3007  -- 结果响应指令码异常
ERR_CAM_DATA_LEN_ERR        = 3008  -- 坐标数据长度不足
ERR_CAM_PHOTO_FAIL          = 3009  -- 拍照失败（重试完）
ERR_CAM_HEIGHT_INVALID_PICK = 3010  -- 取料时高度异常，无法判定层数
ERR_CAM_HEIGHT_INVALID_PLACE= 3011  -- 放料时高度异常，无法判定层数
ERR_CAM_DIR_INVALID_PICK     = 3012  -- 取料时相机方向值非法
ERR_CAM_DIR_INVALID_PLACE    = 3013  -- 放料时相机方向值非法
ERR_MATRIX_RETRY_FAIL_PICK   = 3014  -- 取料获取排列方向失败（重试完）
ERR_MATRIX_RETRY_FAIL_PLACE  = 3015  -- 放料获取排列方向失败（重试完）

-- ============ 4xxx 电芯状态错误 ============
ERR_CELL_PICK_EMPTY         = 4001  -- 取料位未检测到电芯（取料存在检测）
ERR_CELL_GRAB_CONFIRM       = 4002  -- 抓取后夹爪未检测到电芯（取料抓取确认）
ERR_CELL_PLACE_OCCUPIED     = 4003  -- 放料位已有电芯（放料空穴检测）
ERR_CELL_PLACE_RESIDUAL     = 4004  -- 放料后夹爪残留电芯（放料残留检测）
ERR_CELL_MEM_NO_SENSOR_HAS_INIT = 4005  -- 初始化：有抓取记忆但传感器未检测到电芯
ERR_CELL_MEM_NO_SENSOR_HAS_RUN  = 4006  -- 主循环：有抓取记忆但传感器未检测到电芯
ERR_CELL_MEM_HAS_SENSOR_NO_INIT = 4007  -- 初始化：传感器检测到电芯但无抓取记忆
ERR_CELL_MEM_HAS_SENSOR_NO_RUN  = 4008  -- 主循环：传感器检测到电芯但无抓取记忆
ERR_CELL_ALREADY            = 4009  -- 已抓取电芯，禁止重复取料
ERR_CELL_UNREADY            = 4010  -- 未抓取电芯，无法执行放料
ERR_CELL_LAYER_MISMATCH_PICK= 4011  -- 取料时相机判定层数与记忆层数不符
ERR_CELL_LAYER_MISMATCH_PLACE= 4012  -- 放料时相机判定层数与记忆层数不符

-- ============ 5xxx 参数/防呆错误 ============
ERR_PARAM_ROW_INVALID       = 5001  -- 行号非法
ERR_PARAM_COL_INVALID       = 5002  -- 列号非法
ERR_PARAM_LAYER_INVALID     = 5003  -- 层号非法
ERR_PARAM_STATION_INVALID   = 5004  -- 站点号非法
ERR_PARAM_STEP_ZERO         = 5005  -- 矩阵步长为0
ERR_PICK_MATRIX_OVERFLOW    = 5006  -- 取料矩阵参数超限
ERR_PLACE_MEM_MATRIX_OVERFLOW = 5007  -- 放料站点记忆数列超限
ERR_PLACE_UPLOAD_MATRIX_OVERFLOW = 5008  -- 放料上位机强制位置超限
ERR_PRESET_DIR_MISMATCH_PICK= 5009  -- 取料时横纵方向固定启用，与预设定方向不一致
ERR_PRESET_DIR_MISMATCH_PLACE= 5010  -- 放料时横纵方向固定启用，与预设定方向不一致
ERR_SAME_LAYER_DIR_MISMATCH_PICK= 5011  -- 取料时同层方向不一致
ERR_SAME_LAYER_DIR_MISMATCH_PLACE= 5012  -- 放料时同层方向不一致
ERR_PARAM_PLACE_POS_MISMATCH= 5013  -- 放料位置与上位机期望不匹配
ERR_PARAM_PICK_DEV          = 5014  -- 取料坐标偏差超出阈值
ERR_PARAM_PLACE_DEV         = 5015  -- 放料坐标偏差超出阈值

-- 错误码信息表：包含错误描述、所属类别和错误类型
-- 错误类型说明：
--   true  = 暂停类错误（可人工恢复后继续）
--   false = 停止类错误（需人工干预后重启）
local errorMsgTable = {
    [ERR_SYS_NONE]            = {msg = "无错误",                         category = "系统错误",          isPause = true},
    [ERR_SYS_SEARCH_TRIGGER]  = {msg = "寻位安全信号触发，运动中止",      category = "系统错误",          isPause = false},
    [ERR_SYS_ARM_NOT_HOME]    = {msg = "机械臂不在原点且无法自动回原",     category = "系统错误",          isPause = false},
    [ERR_SYS_SCAN_TIMEOUT]    = {msg = "扫码等待超时",                    category = "系统错误",          isPause = true},
    [ERR_SYS_PALLET_TIMEOUT]   = {msg = "等待换盘完成超时",                    category = "系统错误",          isPause = false},
    [ERR_SYS_STACK_TIMEOUT]    = {msg = "等待换垛完成超时",                    category = "系统错误",          isPause = false},
    [ERR_VOLTAGE_TIMEOUT]      = {msg = "测电压等待超时",                    category = "系统错误",          isPause = true},
    [ERR_SCAN_RETRY_FAIL]      = {msg = "扫码等待重试失败",                  category = "系统错误",          isPause = false},
    [ERR_VOLTAGE_RETRY_FAIL]   = {msg = "测电压等待重试失败",                category = "系统错误",          isPause = false},
    [ERR_VOLTAGE_SIGN_TIMEOUT] = {msg = "是否需要测电压信号等待超时",                category = "系统错误",          isPause = true},
    [ERR_SPEED_RATE_INVALID]   = {msg = "速度倍率无效(42001)，已暂停等待有效值",       category = "系统错误",          isPause = true},
    [ERR_PRODUCT_UNDEFINED]   = {msg = "产品号未定义或参数表不合法，已暂停等待修正",   category = "系统错误",          isPause = true},

    [ERR_GRIPPER_OPEN_TIMEOUT] = {msg = "夹爪张开超时",                  category = "夹爪控制错误",      isPause = false},
    [ERR_GRIPPER_CLOSE_TIMEOUT]= {msg = "夹爪闭合超时",                  category = "夹爪控制错误",      isPause = false},
    [ERR_GRIPPER_HOME_TIMEOUT] = {msg = "夹爪回零超时",                  category = "夹爪控制错误",      isPause = false},
    [ERR_GRIPPER_NOT_OPEN]     = {msg = "夹爪不在张开状态",                  category = "夹爪控制错误",      isPause = false},

    [ERR_CAM_CONNECT_FAIL]    = {msg = "相机连接失败",                   category = "相机通讯错误",      isPause = false},
    [ERR_CAM_TRIG_SEND_FAIL]  = {msg = "相机触发指令发送失败",            category = "相机通讯错误",      isPause = true},
    [ERR_CAM_TRIG_NO_RESP]    = {msg = "未接收相机触发响应",              category = "相机通讯错误",      isPause = true},
    [ERR_CAM_TRIG_CODE_ERR]   = {msg = "相机触发响应指令码异常",          category = "相机通讯错误",      isPause = true},
    [ERR_CAM_READ_SEND_FAIL]  = {msg = "相机读取指令发送失败",            category = "相机通讯错误",      isPause = true},
    [ERR_CAM_RESULT_NO_RESP]  = {msg = "未接收相机结果响应",              category = "相机通讯错误",      isPause = true},
    [ERR_CAM_RESULT_CODE_ERR] = {msg = "相机结果响应指令码异常",          category = "相机通讯错误",      isPause = true},
    [ERR_CAM_DATA_LEN_ERR]    = {msg = "相机坐标数据长度不足",            category = "相机通讯错误",      isPause = true},
    [ERR_CAM_PHOTO_FAIL]      = {msg = "相机拍照失败",                   category = "相机通讯错误",      isPause = false},
    [ERR_CAM_HEIGHT_INVALID_PICK]= {msg = "取料时高度异常，无法判定层数",     category = "相机通讯错误",      isPause = true},
    [ERR_CAM_HEIGHT_INVALID_PLACE]={msg = "放料时高度异常，无法判定层数",      category = "相机通讯错误",      isPause = true},
    [ERR_CAM_DIR_INVALID_PICK]   = {msg = "取料时相机方向值非法",            category = "相机通讯错误",      isPause = true},
    [ERR_CAM_DIR_INVALID_PLACE]  = {msg = "放料时相机方向值非法",            category = "相机通讯错误",      isPause = true},
    [ERR_MATRIX_RETRY_FAIL_PICK]   = {msg = "取料获取排列方向重试失败",      category = "相机通讯错误",      isPause = false},
    [ERR_MATRIX_RETRY_FAIL_PLACE]  = {msg = "放料获取排列方向重试失败",      category = "相机通讯错误",      isPause = false},

    [ERR_CELL_PICK_EMPTY]      = {msg = "取料位未检测到电芯",                category = "电芯状态错误",      isPause = false},
    [ERR_CELL_GRAB_CONFIRM]    = {msg = "抓取后夹爪未检测到电芯",            category = "电芯状态错误",      isPause = false},
    [ERR_CELL_PLACE_OCCUPIED]  = {msg = "放料位已有电芯，无法放置",          category = "电芯状态错误",      isPause = false},
    [ERR_CELL_PLACE_RESIDUAL]  = {msg = "放料后夹爪残留电芯",                category = "电芯状态错误",      isPause = false},
    [ERR_CELL_MEM_NO_SENSOR_HAS_INIT]={msg = "初始化：有抓取记忆但传感器未检测到电芯", category = "电芯状态错误", isPause = false},
    [ERR_CELL_MEM_NO_SENSOR_HAS_RUN]={msg = "主循环：有抓取记忆但传感器未检测到电芯",  category = "电芯状态错误", isPause = false},
    [ERR_CELL_MEM_HAS_SENSOR_NO_INIT]={msg = "初始化：传感器检测到电芯但无抓取记忆",   category = "电芯状态错误", isPause = false},
    [ERR_CELL_MEM_HAS_SENSOR_NO_RUN]={msg = "主循环：传感器检测到电芯但无抓取记忆",    category = "电芯状态错误", isPause = false},
    [ERR_CELL_ALREADY]        = {msg = "已抓取电芯，禁止重复取料",            category = "电芯状态错误",      isPause = false},
    [ERR_CELL_UNREADY]        = {msg = "未抓取电芯，无法执行放料",            category = "电芯状态错误",      isPause = false},
    [ERR_CELL_LAYER_MISMATCH_PICK]={msg = "取料时相机判定层数与记忆层数不符",  category = "电芯状态错误",      isPause = false},
    [ERR_CELL_LAYER_MISMATCH_PLACE]={msg = "放料时相机判定层数与记忆层数不符", category = "电芯状态错误",      isPause = false},

    [ERR_PARAM_ROW_INVALID]   = {msg = "行号非法",                       category = "参数/防呆错误",     isPause = false},
    [ERR_PARAM_COL_INVALID]   = {msg = "列号非法",                       category = "参数/防呆错误",     isPause = false},
    [ERR_PARAM_LAYER_INVALID] = {msg = "层号非法",                       category = "参数/防呆错误",     isPause = false},
    [ERR_PARAM_STATION_INVALID] = {msg = "站点号非法",                   category = "参数/防呆错误",     isPause = false},
    [ERR_PARAM_STEP_ZERO]     = {msg = "矩阵步长为0",                    category = "参数/防呆错误",     isPause = false},
    [ERR_PICK_MATRIX_OVERFLOW]= {msg = "取料矩阵参数超限",               category = "参数/防呆错误",     isPause = false},
    [ERR_PLACE_MEM_MATRIX_OVERFLOW] = {msg = "放料站点记忆矩阵超限",     category = "参数/防呆错误",     isPause = false},
    [ERR_PLACE_UPLOAD_MATRIX_OVERFLOW] = {msg = "放料上位机强制位置超限", category = "参数/防呆错误",     isPause = false},
    [ERR_PRESET_DIR_MISMATCH_PICK]= {msg = "取料时横纵方向与预设定方向不一致",    category = "参数/防呆错误",     isPause = false},
    [ERR_PRESET_DIR_MISMATCH_PLACE]={msg = "放料时横纵方向与预设定方向不一致",     category = "参数/防呆错误",     isPause = false},
    [ERR_SAME_LAYER_DIR_MISMATCH_PICK]={msg = "取料时同层方向不一致",            category = "参数/防呆错误",     isPause = false},
    [ERR_SAME_LAYER_DIR_MISMATCH_PLACE]={msg = "放料时同层方向不一致",           category = "参数/防呆错误",     isPause = false},
    [ERR_PARAM_PLACE_POS_MISMATCH]= {msg = "放料位置与上位机期望不匹配", category = "参数/防呆错误",     isPause = false},
    [ERR_PARAM_PICK_DEV]      = {msg = "取料坐标偏差超出阈值",           category = "参数/防呆错误",     isPause = false},
    [ERR_PARAM_PLACE_DEV]     = {msg = "放料坐标偏差超出阈值",           category = "参数/防呆错误",     isPause = false}
}

--[[
函数名：getErrorMsg
功能：获取错误码中文描述
参数：errCode - 错误码编号
返回：错误描述字符串
]]
function getErrorMsg(errCode)
    local info = errorMsgTable[errCode]
    return info and info.msg or "未知错误"
end

--[[
函数名：setError
功能：设置错误状态，向上位机反馈错误码，并根据错误类型自动执行暂停或停止
参数：errCode - 错误码编号
返回：无
]]
function setError(errCode)
	-- 缓存各状态地址当前状态值
	local armStatus = getRegisterInt(REG_ARM_STAT)
	local robotStatus = getRegisterInt(REG_ROBOT_STAT)
	local cmdAck = getRegisterInt(REG_CMD_ACK)

    local errInfo = errorMsgTable[errCode]
    if errInfo == nil then
        errInfo = {msg = "未知错误", category = "未知类别", isPause = true}
    end
	-- 置位各状态地址错误状态值
    setRegisterInt(REG_ERROR_CODE, errCode)     -- 写入错误码
    setRegisterInt(REG_ARM_STAT, 3)             -- 机械臂状态设为错误
    setRegisterInt(REG_ROBOT_STAT, 3)           -- 机器人整体状态设为错误
    setRegisterInt(REG_CMD_ACK, -errCode)       -- 指令应答返回负错误码

    local errTypeStr = errInfo.isPause and "暂停可恢复" or "停止需复位"
    local errMsg = string.format("[错误] 错误码：%d | 类别：%s | 类型：%s | 描述：%s",
                                 errCode, errInfo.category, errTypeStr, errInfo.msg)
    print(errMsg)

    -- wait(20)  -- 等待错误信息稳定

    if errInfo.isPause then
        print("[提示] 此错误为暂停类型，人工处理后请恢复程序继续运行")
        pauseProject()
		-- 暂停复位继续，则恢复各状态地址状态值
		setRegisterInt(REG_ERROR_CODE, ERR_SYS_NONE)   -- 清除错误码
		setRegisterInt(REG_ARM_STAT, armStatus)        -- 机械臂状态恢复
		setRegisterInt(REG_ROBOT_STAT, robotStatus)    -- 机器人整体状态恢复
		setRegisterInt(REG_CMD_ACK, cmdAck)            -- 指令应答恢复
    else
        print("[严重] 此错误为停止类型，需要人工干预后重启程序")
        stopProject()
    end
end

--[[
函数名：clearError
功能：清除所有错误状态，恢复空闲
参数：无
返回：无
]]
function clearError()
    setRegisterInt(REG_ERROR_CODE, ERR_SYS_NONE)   -- 清除错误码
    setRegisterInt(REG_ARM_STAT, 0)                -- 机械臂状态复位
    setRegisterInt(REG_ROBOT_STAT, 0)              -- 机器人整体状态复位
    setRegisterInt(REG_CMD_ACK, 0)                 -- 指令应答复位
    print("[系统] 错误状态已清除")
end

--[[
函数名：waitWithTimeout
功能：带超时的条件等待
参数：
  condFn      - 条件判断函数
  timeoutMs   - 超时时间(ms)，默认30000
  intervalMs  - 轮询间隔(ms)，默认POLL_INTERVAL
  hint        - 超时提示文本
返回：bool true=条件满足 false=超时
]]
function waitWithTimeout(condFn, timeoutMs, intervalMs, hint)
    timeoutMs  = timeoutMs or 30000     -- 默认超时时间30秒
    intervalMs = intervalMs or POLL_INTERVAL  -- 默认轮询间隔
    local elapsed = 0                   -- 已等待时间

    while elapsed < timeoutMs do
        if condFn() then
            return true                 -- 条件满足，返回true
        end
        wait(intervalMs)                -- 等待一个轮询间隔
        elapsed = elapsed + intervalMs  -- 累加已等待时间
    end

    if hint then
        print("[超时警告] " .. hint)
    end
    return false                        -- 超时返回false
end

--[[
函数名：checkParamValid
功能：参数合法性防呆校验
参数：
  row     - 行号
  col     - 列号
  layer   - 层号
  station - 站点号（可选）
返回：bool true=合法 false=非法
]]
function checkParamValid(row, col, layer, station)
    -- 行号正整数校验
    if type(row) ~= "number" or row ~= math.floor(row) or row < 1 then
print("[参数防呆] 行号非法: " .. tostring(row))
        setError(ERR_PARAM_ROW_INVALID)
        return false
    end
    -- 列号正整数校验
    if type(col) ~= "number" or col ~= math.floor(col) or col < 1 then
        print("[参数防呆] 列号非法: " .. tostring(col))
        setError(ERR_PARAM_COL_INVALID)
        return false
    end
    -- 层号正整数校验
    if type(layer) ~= "number" or layer ~= math.floor(layer) or layer < 1 then
        print("[参数防呆] 层号非法: " .. tostring(layer))
        setError(ERR_PARAM_LAYER_INVALID)
        return false
    end
    
    -- 站点号校验（可选参数）
    if station ~= nil then
        if type(station) ~= "number" or station ~= math.floor(station)
            or station < 1 or station > MAX_STATION then
            print("[参数防呆] 站点号非法: " .. tostring(station))
            setError(ERR_PARAM_STATION_INVALID)
            return false
        end
    end
    
    -- 步长参数非0校验
    if PICK_STEP_X == 0 or PICK_STEP_Y == 0 or PLACE_STEP_X == 0 or PLACE_STEP_Y == 0 then
        print("[参数防呆] 矩阵步长不能为0")
        setError(ERR_PARAM_STEP_ZERO)
        return false
    end

    return true  -- 所有参数校验通过
end

--[[
函数名：movLWithSearch
功能：带寻位检测的直线运动，触发DI信号后自动抬升并暂停
参数：
  targetPose - 目标位姿
  opt        - 运动参数表（v/a等），可选
返回：bool true=正常到达 false=寻位触发
]]
function movLWithSearch(targetPose, opt)
    local moveOpt = opt or {}
    -- 设置寻位检测条件：DI信号为1时触发
    moveOpt.search = string.format("getDI(%d) == 1", SEARCH_IO_NUM)

    -- 执行带寻位的直线运动
    movL(targetPose, moveOpt)

    -- 检测寻位信号是否触发
    if getDI(SEARCH_IO_NUM) == 1 then
        -- 沿Z轴正方向抬升安全距离
        setMoveRate(30)
        local safePose = coorRel(getTCP(), {0, 0, SEARCH_LIFT_HEIGHT, 0, 0, 0})
        movL(safePose)

        -- 打印当前位置信息
        local curPos = getTCP()
        print(string.format("[寻位触发] 已抬升%.1fmm | 当前位置 X:%.2f Y:%.2f Z:%.2f",
            SEARCH_LIFT_HEIGHT, curPos.cp[1], curPos.cp[2], curPos.cp[3]))

        -- 设置寻位触发错误
        setError(ERR_SYS_SEARCH_TRIGGER)
        return false
    end
    return true  -- 正常到达目标位置
end

--[[
函数名：checkPickDeviation
功能：取料坐标偏差校验，仅(1,1)点位记录该层基准，其余点位按矩阵偏移计算理论值并校验偏差
参数：
  layer    - 层号
  curPose  - 当前相机反馈的实际抓取位姿
  dx_theo  - 当前点相对于基准点的X方向理论偏移量
  dy_theo  - 当前点相对于基准点的Y方向理论偏移量
  row      - 当前行号
  col      - 当前列号
返回：bool true=偏差正常 false=超限
]]
function checkPickDeviation(layer, curPose, dx_theo, dy_theo, row, col)
    -- 仅(1,1)点位允许记录基准
    if not g_pickBasePose[layer] then
        if row == 1 and col == 1 then
            -- 记录该层基准位姿
            g_pickBasePose[layer] = {
                cp = {
                    curPose.cp[1], curPose.cp[2], curPose.cp[3],
                    curPose.cp[4], curPose.cp[5], curPose.cp[6]
                }
            }
            print(string.format("[取料基准] 第%d层基准已记录 | X:%.2f Y:%.2f Z:%.2f RX:%.2f RY:%.2f RZ:%.2f",
                layer, curPose.cp[1], curPose.cp[2], curPose.cp[3],
                curPose.cp[4], curPose.cp[5], curPose.cp[6]))
            return true
        else
            print(string.format("[校验跳过] 第%d层未记录基准，非(1,1)点位，跳过偏差校验", layer))
            return true
        end
    end

    local base = g_pickBasePose[layer]
    -- 计算理论位姿：基准 + XY矩阵偏移，Z与姿态同基准
    local theo_x = base.cp[1] + dx_theo
    local theo_y = base.cp[2] + dy_theo
    local theo_z = base.cp[3]
    local theo_rx = base.cp[4]
    local theo_ry = base.cp[5]
    local theo_rz = base.cp[6]

    -- 打印理论值与实际值
    print(string.format("[取料理论点] 第%d层 行%d列%d | X:%.2f Y:%.2f Z:%.2f RX:%.2f RY:%.2f RZ:%.2f",
        layer, row, col, theo_x, theo_y, theo_z, theo_rx, theo_ry, theo_rz))
    print(string.format("[取料实际点] 第%d层 行%d列%d | X:%.2f Y:%.2f Z:%.2f RX:%.2f RY:%.2f RZ:%.2f",
        layer, row, col, curPose.cp[1], curPose.cp[2], curPose.cp[3],
        curPose.cp[4], curPose.cp[5], curPose.cp[6]))

    -- 计算6轴偏差
    local dx = curPose.cp[1] - theo_x
    local dy = curPose.cp[2] - theo_y
    local dz = curPose.cp[3] - theo_z
    local drx = curPose.cp[4] - theo_rx
    local dry = curPose.cp[5] - theo_ry
    local drz = curPose.cp[6] - theo_rz

    print(string.format("[取料偏差值] dx:%.3f dy:%.3f dz:%.3f | drx:%.3f dry:%.3f drz:%.3f",
        dx, dy, dz, drx, dry, drz))

    -- 分项阈值判断
    local dev_over = false
    if math.abs(dx) > POSE_DEV_XYZ_THRESHOLD then dev_over = true end
    if math.abs(dy) > POSE_DEV_XYZ_THRESHOLD then dev_over = true end
    if math.abs(dz) > POSE_DEV_XYZ_THRESHOLD then dev_over = true end
    if math.abs(drx) > POSE_DEV_R_THRESHOLD then dev_over = true end
    if math.abs(dry) > POSE_DEV_R_THRESHOLD then dev_over = true end
    if math.abs(drz) > POSE_DEV_R_THRESHOLD then dev_over = true end

    if dev_over then
        print(string.format("[取料偏差超限] 位置阈值:%.2fmm 旋转阈值:%.2f°",
            POSE_DEV_XYZ_THRESHOLD, POSE_DEV_R_THRESHOLD))
        setError(ERR_PARAM_PICK_DEV)
        return false
    end
    return true
end

--[[
函数名：checkPlaceDeviation
功能：放料坐标偏差校验，每层第1个电芯记录站点+层基准，其余点位按矩阵偏移计算理论值并校验偏差
参数：
  station  - 站点号
  layer    - 层号
  curPose  - 当前相机反馈的实际放料位姿
  dx_theo  - 当前点相对于基准点的X方向理论偏移量
  dy_theo  - 当前点相对于基准点的Y方向理论偏移量
  cur_count - 当前电芯序号（该层第几个）
返回：bool true=偏差正常 false=超限
]]
function checkPlaceDeviation(station, layer, curPose, dx_theo, dy_theo, cur_count)
    local key = string.format("%d_%d", station, layer)  -- 站点+层复合key

    -- 仅第1个电芯允许记录基准
    if not g_placeBasePose[key] then
        if cur_count == 1 then
            -- 倒序放置时，需要反推基准点位置
            if PLACE_REVERSE_ORDER then
                curPose.cp[1] = curPose.cp[1] - dx_theo
                curPose.cp[2] = curPose.cp[2] - dy_theo
            end
            -- 记录该站点该层基准位姿
            g_placeBasePose[key] = {
                cp = {
                    curPose.cp[1], curPose.cp[2], curPose.cp[3],
                    curPose.cp[4], curPose.cp[5], curPose.cp[6]
                }
            }
            print(string.format("[放料基准] 站点%d第%d层基准已记录 | X:%.2f Y:%.2f Z:%.2f RX:%.2f RY:%.2f RZ:%.2f",
                station, layer, curPose.cp[1], curPose.cp[2], curPose.cp[3],
                curPose.cp[4], curPose.cp[5], curPose.cp[6]))
            return true
        else
            print(string.format("[校验跳过] 站点%d第%d层未记录基准，非第1个电芯，跳过偏差校验", station, layer))
            return true
        end
    end

    local base = g_placeBasePose[key]
    -- 计算理论位姿：基准 + XY矩阵偏移，Z与姿态同基准
    local theo_x = base.cp[1] + dx_theo
    local theo_y = base.cp[2] + dy_theo
    local theo_z = base.cp[3]
    local theo_rx = base.cp[4]
    local theo_ry = base.cp[5]
    local theo_rz = base.cp[6]

    -- 打印理论值与实际值
    print(string.format("[放料理论点] 站点%d第%d层 第%d个 | X:%.2f Y:%.2f Z:%.2f RX:%.2f RY:%.2f RZ:%.2f",
        station, layer, cur_count, theo_x, theo_y, theo_z, theo_rx, theo_ry, theo_rz))
    print(string.format("[放料实际点] 站点%d第%d层 第%d个 | X:%.2f Y:%.2f Z:%.2f RX:%.2f RY:%.2f RZ:%.2f",
        station, layer, cur_count, curPose.cp[1], curPose.cp[2], curPose.cp[3],
        curPose.cp[4], curPose.cp[5], curPose.cp[6]))

    -- 计算6轴偏差
    local dx = curPose.cp[1] - theo_x
    local dy = curPose.cp[2] - theo_y
    local dz = curPose.cp[3] - theo_z
    local drx = curPose.cp[4] - theo_rx
    local dry = curPose.cp[5] - theo_ry
    local drz = curPose.cp[6] - theo_rz

    print(string.format("[放料偏差值] dx:%.3f dy:%.3f dz:%.3f | drx:%.3f dry:%.3f drz:%.3f",
        dx, dy, dz, drx, dry, drz))

    -- 分项阈值判断
    local dev_over = false
    if math.abs(dx) > POSE_DEV_XYZ_THRESHOLD then dev_over = true end
    if math.abs(dy) > POSE_DEV_XYZ_THRESHOLD then dev_over = true end
    if math.abs(dz) > POSE_DEV_XYZ_THRESHOLD then dev_over = true end
    if math.abs(drx) > POSE_DEV_R_THRESHOLD then dev_over = true end
    if math.abs(dry) > POSE_DEV_R_THRESHOLD then dev_over = true end
    if math.abs(drz) > POSE_DEV_R_THRESHOLD then dev_over = true end

    if dev_over then
        print(string.format("[放料偏差超限] 位置阈值:%.2fmm 旋转阈值:%.2f°",
            POSE_DEV_XYZ_THRESHOLD, POSE_DEV_R_THRESHOLD))
        setError(ERR_PARAM_PLACE_DEV)
        return false
    end
    return true
end

-- ===================== 第六部分：多站点状态管理 =====================
--[[
函数名：initStationState
功能：初始化指定站点状态
参数：station - 站点号
返回：无
]]
function initStationState(station)
    if not g_stationState[station] then
        -- 初始化站点状态表
        g_stationState[station] = {
            stationNum = station,       -- 站点号
            count      = 0,             -- 已放电芯数量
            side       = 0,             -- 站点侧位
            layer      = 1,             -- 当前层号
            cellMatrix = nil,           -- 电芯排列方向
            palletFull = false,         -- 托盘满标志
            stackFull  = false,         -- 料垛满标志
            lastMatrix = nil,           -- 上一次排列方向
            lastLayer  = nil            -- 上一次层号
        }
        print(string.format("[站点管理] 初始化站点%d状态", station))
    end
end

--[[
函数名：getPlaceSide
功能：获取站点侧位与盘满状态
参数：station - 站点号
返回：side, palletFull
]]
function getPlaceSide(station)
    initStationState(station)
    local st = g_stationState[station]
    -- 判断托盘是否已满（按当前产品规格）
    local _, _, _, trayTotal = getTrayDims()
    st.palletFull = st.count >= trayTotal
    return st.side, st.palletFull
end

--[[
函数名：incStationCount
功能：站点已放电芯计数+1
参数：station - 站点号
返回：无
]]
function incStationCount(station)
    initStationState(station)
    g_stationState[station].count = g_stationState[station].count + 1
    local _, _, _, trayTotal = getTrayDims()
    print(string.format("[站点管理] 站点%d已放: %d/%d",
        station, g_stationState[station].count, trayTotal))
end

-- ===================== 第七部分：相机通讯与坐标处理 =====================
--[[
函数名：initCameraSocket
功能：初始化相机TCP连接，支持重试
返回：bool true=连接成功
]]
function initCameraSocket()
    if g_cameraConnected then
        print("[相机通讯] 已连接，跳过初始化")
        return true
    end

    local retry = 0
    -- 循环尝试连接相机
    while retry < RETRY_MAX do
        print(string.format("[相机通讯] 第%d次尝试连接 %s:%d",
            retry + 1, CAMERA_IP, CAMERA_PORT))

        -- 创建Socket客户端
        createSocketClient("camera", CAMERA_IP, CAMERA_PORT)
        wait(500)

        -- 尝试建立连接
        local connRet = connectSocketClient("camera", SOCKET_TIMEOUT)
        if connRet == 0 then
            g_cameraConnected = true
            print("[相机通讯] 连接成功")
            return true
        end

        -- 连接失败，关闭并重试
        closeSocketClient("camera")
        retry = retry + 1
        wait(500)
    end

    print(string.format("[相机通讯] 连接失败，已重试%d次", RETRY_MAX))
    setError(ERR_CAM_CONNECT_FAIL)
    return false
end

--[[
函数名：camera_comm
功能：触发相机拍照并解析坐标
参数：
  camera_id   - 拍照工程ID
  tcp_pose    - 拍照位姿
  stationSide - 站点侧位
返回：sPointArr 坐标数组
]]
function camera_comm(camera_id, tcp_pose, stationSide)
    print("=== 相机拍照流程开始 ===")

    -- 移动到拍照过渡点
    -- 过渡点加b=60与拍照位运动圆滑衔接（拐角不停站，拍照位仍精确停止；v3.4.8）
    -- 注意：b过渡要求两条运动指令相邻，中间不能插入print等语句
    if stationSide == 0 then
        movJ(GD_ZC, {v = 120, a = 200, b = 60})  -- 左侧过渡点
    else
        movJ(GD_YC, {v = 120, a = 200, b = 60})  -- 右侧过渡点
    end
    movJ(tcp_pose)  -- 移动到拍照位
	print("当前拍照位关节: ",getJoint())
    local retry = 0
    ::CAM_RETRY::
    -- clearError()  -- 清除历史错误
    wait(500)

    while retry < RETRY_MAX do
        -- 构造触发指令：101,工程ID,0,2,当前TCP坐标
        local triggerCmd = string.format("101,%d,0,2,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
            camera_id,
            getTCP({tool=0}).cp[1], getTCP({tool=0}).cp[2], getTCP({tool=0}).cp[3],
            getTCP({tool=0}).cp[4], getTCP({tool=0}).cp[5], getTCP({tool=0}).cp[6])

        print("[相机指令] 发送触发指令: " .. triggerCmd)

        -- 发送触发指令
        local writeRet = writeSocketClient("camera", triggerCmd, SOCKET_TIMEOUT)
        if writeRet ~= 0 then
            print("[相机通讯] 触发指令发送失败，错误码: " .. writeRet)
            setError(ERR_CAM_TRIG_SEND_FAIL)
            closeSocketClient("camera")
            g_cameraConnected = false
            initCameraSocket()
            retry = retry + 1
            goto CAM_RETRY
        end
        wait(20)

        -- 读取触发响应（带失败次数上限，防止通讯异常时无限循环）
        local readRet, acceptData
        local readFail = 0
        repeat
            readRet, acceptData = readSocketClient("camera", SOCKET_TIMEOUT)
            if readRet ~= 0 then
                readFail = readFail + 1
                print(string.format("[相机通讯] 读取失败(ret=%s) 第%d次", tostring(readRet), readFail))
            end
        until readRet == 0 or readFail >= RETRY_MAX
        if readRet ~= 0 then
            print("[相机通讯] 读取多次失败，转入重连重试")
            setError(ERR_CAM_TRIG_NO_RESP)
            closeSocketClient("camera")
            g_cameraConnected = false
            initCameraSocket()
            retry = retry + 1
            goto CAM_RETRY
        end

        if not acceptData then
            print("[相机通讯] 未接收到触发响应")
            setError(ERR_CAM_TRIG_NO_RESP)
            closeSocketClient("camera")
            g_cameraConnected = false
            initCameraSocket()
            retry = retry + 1
            goto CAM_RETRY
        end
        print("[相机响应] 触发响应: " .. acceptData)

        -- 校验响应指令码（应为1102）
        local respParts = strToNumberArray(acceptData, {sep = ","})
        if #respParts < 2 or respParts[2] ~= 1102 then
            print("[相机通讯] 触发响应指令码异常")
            setError(ERR_CAM_TRIG_CODE_ERR)
            retry = retry + 1
            goto CAM_RETRY
        end
        wait(20)

        -- 发送读取结果指令：102,工程ID
        local readCmd = string.format("102,%d", camera_id)
        writeRet = writeSocketClient("camera", readCmd, SOCKET_TIMEOUT)
        if writeRet ~= 0 then
            print("[相机通讯] 读取指令发送失败")
            setError(ERR_CAM_READ_SEND_FAIL)
            closeSocketClient("camera")
            g_cameraConnected = false
            initCameraSocket()
            retry = retry + 1
            goto CAM_RETRY
        end
        print("[相机指令] 发送读取指令: " .. readCmd)
        wait(20)

        -- 读取结果响应（带失败次数上限，防止通讯异常时无限循环）
        local readFail = 0
        repeat
            readRet, acceptData = readSocketClient("camera", SOCKET_TIMEOUT)
            if readRet ~= 0 then
                readFail = readFail + 1
                print(string.format("[相机通讯] 读取失败(ret=%s) 第%d次", tostring(readRet), readFail))
            end
        until readRet == 0 or readFail >= RETRY_MAX
        if readRet ~= 0 then
            print("[相机通讯] 读取多次失败，转入重连重试")
            setError(ERR_CAM_RESULT_NO_RESP)
            closeSocketClient("camera")
            g_cameraConnected = false
            initCameraSocket()
            retry = retry + 1
            goto CAM_RETRY
        end

        if not acceptData then
            print("[相机通讯] 未接收到结果响应")
            setError(ERR_CAM_RESULT_NO_RESP)
            closeSocketClient("camera")
            g_cameraConnected = false
            initCameraSocket()
            retry = retry + 1
            goto CAM_RETRY
        end
        print("[相机响应] 结果响应: " .. acceptData)

        -- 校验结果指令码（应为1100）
        respParts = strToNumberArray(acceptData, {sep = ","})
        if #respParts < 2 or respParts[2] ~= 1100 then
			if respParts[2] and respParts[2] > 6000 then
				-- 缓存各状态地址当前状态值
				local armStatus = getRegisterInt(REG_ARM_STAT)
				local robotStatus = getRegisterInt(REG_ROBOT_STAT)
				local cmdAck = getRegisterInt(REG_CMD_ACK)
				print("[相机通讯] 结果响应指令码异常")
		        -- 置位各状态地址错误状态值
				setRegisterInt(REG_ERROR_CODE, respParts[2])     -- 写入错误码
				setRegisterInt(REG_ARM_STAT, 3)             -- 机械臂状态设为错误
				setRegisterInt(REG_ROBOT_STAT, 3)           -- 机器人整体状态设为错误
				setRegisterInt(REG_CMD_ACK, -respParts[2])       -- 指令应答返回负错误码
				-- 暂停程序，等待排查
				pauseProject()
				-- 暂停复位继续，则恢复各状态地址状态值
				setRegisterInt(REG_ERROR_CODE, ERR_SYS_NONE)   -- 清除错误码
				setRegisterInt(REG_ARM_STAT, armStatus)        -- 机械臂状态恢复
				setRegisterInt(REG_ROBOT_STAT, robotStatus)    -- 机器人整体状态恢复
				setRegisterInt(REG_CMD_ACK, cmdAck)            -- 指令应答恢复
			else
				setError(ERR_CAM_RESULT_CODE_ERR)
			end
            retry = retry + 1
            goto CAM_RETRY
        end
        wait(20)

        -- 解析坐标数据
        local sPointArr = strToNumberArray(acceptData, {sep = ","})
        if #sPointArr < 12 then
            print(string.format("[相机通讯] 坐标数据长度不足: %d", #sPointArr))
            setError(ERR_CAM_DATA_LEN_ERR)
            retry = retry + 1
            goto CAM_RETRY
        end

        -- 打印解析结果
        print(string.format("[相机解析] X:%.2f Y:%.2f Z:%.2f Rx:%.2f Ry:%.2f Rz:%.2f",
            sPointArr[6], sPointArr[7], sPointArr[8],
            sPointArr[9], sPointArr[10], sPointArr[11]))
        print("=== 相机拍照流程结束 ===")
        return sPointArr
    end

    print(string.format("[相机通讯] 拍照失败，已重试%d次", RETRY_MAX))
    setError(ERR_CAM_PHOTO_FAIL)
    return nil
end

--[[
函数名：applyCameraPose
功能：将相机坐标应用到目标位姿变量
参数：
  targetPose - 目标位姿变量
  sPointArr  - 相机返回坐标数组
返回：无
]]
function applyCameraPose(targetPose, sPointArr)
    -- 更新目标位姿的6个分量
    targetPose.cp[1] = sPointArr[6]   -- X坐标
    targetPose.cp[2] = sPointArr[7]   -- Y坐标
    targetPose.cp[3] = sPointArr[8]   -- Z坐标
    targetPose.cp[4] = sPointArr[9]   -- RX旋转
    targetPose.cp[5] = sPointArr[10]  -- RY旋转
    targetPose.cp[6] = sPointArr[11]  -- RZ旋转
    print("[坐标应用] 目标点位已更新")
end

-- ===================== 第八部分：电芯矩阵偏移计算 =====================
--[[
函数名：calcPickOffset
功能：计算取料拍照位矩阵偏移
参数：
  row, col, layer - 行列层号
  basePose        - 基准位姿
  stepX, stepY, stepZ - 各方向步长
返回：偏移后位姿
]]
function calcPickOffset(row, col, layer, basePose, stepX, stepY, stepZ)
    -- 复制基准位姿
    local newPose = {
        cp = {
            basePose.cp[1], basePose.cp[2], basePose.cp[3] + CUR_PRODUCT.camZShift,  -- 拍照基准Z按产品平移
            basePose.cp[4], basePose.cp[5], basePose.cp[6]
        }
    }
    print(string.format("[取料偏移] 基准点 X:%.2f Y:%.2f Z:%.2f RX:%.2f RY:%.2f RZ:%.2f",
        newPose.cp[1], newPose.cp[2], newPose.cp[3], newPose.cp[4], newPose.cp[5], newPose.cp[6]))

    -- 根据排列方向计算偏移
    if g_cellMatrix == 0 then
        -- X优先：列方向偏移X，行方向偏移Y
        newPose.cp[1] = newPose.cp[1] + (col - 1) * stepX
        newPose.cp[2] = newPose.cp[2] + (row - 1) * stepY
    else
        -- Y优先：行方向偏移X，列方向偏移Y
        newPose.cp[1] = newPose.cp[1] + (row - 1) * stepY
        newPose.cp[2] = newPose.cp[2] + (col - 1) * stepX
    end

    -- Z方向偏移（第4层跳过）
    if layer ~= 4 then
        newPose.cp[3] = newPose.cp[3] - (MAX_LAYER - layer) * stepZ
    end

    print(string.format("[取料偏移] 偏移后 X:%.2f Y:%.2f Z:%.2f RX:%.2f RY:%.2f RZ:%.2f",
        newPose.cp[1], newPose.cp[2], newPose.cp[3], newPose.cp[4], newPose.cp[5], newPose.cp[6]))
    return newPose
end

--[[
函数名：calcPlaceOffset
功能：计算放料拍照位矩阵偏移，支持正/倒序
参数：
  direction   - 排列方向 0=X优先 1=Y优先
  index       - 电芯序号
  layer       - 层号
  basePose    - 基准位姿
  stepX/Y/Z   - 步长
  reverseOrder - 是否倒序，默认全局配置
返回：偏移后位姿
]]
function calcPlaceOffset(direction, index, layer, basePose, stepX, stepY, stepZ, reverseOrder)
    -- 显式false必须生效（原and/or写法会把false吞掉回退全局值，v3.4.5修正）
    if reverseOrder == nil then reverseOrder = PLACE_REVERSE_ORDER end

    -- 获取当前托盘实际行列数（单一事实源：行列表寄存器）
    local curMaxRow, curMaxCol, _, curTotal = getTrayDims()

    -- 倒序转换
    local realIndex = index
    if reverseOrder then
        realIndex = curTotal - index + 1
        print(string.format("[放料偏移] 倒序转换 原序号:%d → 实际序号:%d", index, realIndex))
    end

    -- 转换为行列号
    local row = math.floor((realIndex - 1) / curMaxCol) + 1
    local col = (realIndex - 1) % curMaxCol + 1
    print(string.format("[放料偏移] 对应行列 行:%d 列:%d", row, col))

    -- 保存当前行列号（用于误差补偿）
    g_PlaceRow = row
    g_PlaceCol = col

    -- 复制基准位姿
    local newPose = {
        cp = {
            basePose.cp[1], basePose.cp[2], basePose.cp[3] + CUR_PRODUCT.camZShift,  -- 拍照基准Z按产品平移
            basePose.cp[4], basePose.cp[5], basePose.cp[6]
        }
    }
    print(string.format("[放料偏移] 基准点 X:%.2f Y:%.2f Z:%.2f RX:%.2f RY:%.2f RZ:%.2f",
        newPose.cp[1], newPose.cp[2], newPose.cp[3], newPose.cp[4], newPose.cp[5], newPose.cp[6]))

    -- XY方向偏移
    if direction == 0 then
        -- X优先：列方向偏移X，行方向偏移Y
        newPose.cp[1] = newPose.cp[1] + (col - 1) * stepX
        newPose.cp[2] = newPose.cp[2] + (row - 1) * stepY
    else
        -- Y优先：行方向偏移X，列方向偏移Y
        newPose.cp[1] = newPose.cp[1] + (row - 1) * stepY
        newPose.cp[2] = newPose.cp[2] + (col - 1) * stepX
    end

    -- Z方向偏移（第4层跳过）
    if layer ~= 4 then
        newPose.cp[3] = newPose.cp[3] - (MAX_LAYER - layer) * stepZ
    end

    print(string.format("[放料偏移] 偏移后 X:%.2f Y:%.2f Z:%.2f RX:%.2f RY:%.2f RZ:%.2f",
        newPose.cp[1], newPose.cp[2], newPose.cp[3], newPose.cp[4], newPose.cp[5], newPose.cp[6]))
    return newPose
end

-- ===================== 第九部分：夹爪控制函数 =====================
--[[
函数名：gripperOpen
功能：张开夹爪，带到位检测
参数：无
返回：无
]]
function gripperOpen()
    print("[夹爪控制] 执行张开动作")

    -- 输出张开信号
    setDO(GRIPPER_CLOSE_OUT, 0)  -- 关闭闭合输出
    setDO(GRIPPER_OPEN_OUT, 1)   -- 打开张开输出

    -- 等待张开到位
    local ok = waitWithTimeout(
        function()
            return getDI(GRIPPER_CLOSE_IN) == 1 and getDI(GRIPPER_OPEN_IN) == 1
        end,
        3000, 50, "夹爪张开超时")

    -- 关闭输出
    setDO(GRIPPER_CLOSE_OUT, 0)
    setDO(GRIPPER_OPEN_OUT, 0)

if not ok then
        setError(ERR_GRIPPER_OPEN_TIMEOUT)  -- 设置夹爪张开超时错误
        return
    end
    
    -- 更新状态
    setRegisterBool(COIL_GRIPPER_OPEN, 1)   -- 夹爪张开反馈
    setRegisterBool(COIL_GRIPPER_CLOSE, 0)  -- 夹爪闭合反馈
    g_cellGrabbed = false                    -- 更新抓取标志
    print("[夹爪控制] 张开完成")
end

--[[
函数名：gripperClose
功能：闭合夹爪抓取电芯，带到位检测
参数：无
返回：无
]]
function gripperClose()
    print("[夹爪控制] 执行闭合动作")

    -- 输出闭合信号
    setDO(GRIPPER_CLOSE_OUT, 1)  -- 打开闭合输出
    setDO(GRIPPER_OPEN_OUT, 0)   -- 关闭张开输出

    -- 等待闭合到位
    local ok = waitWithTimeout(
        function()
            return getDI(GRIPPER_CLOSE_IN) == 1 and getDI(GRIPPER_OPEN_IN) == 0
        end,
        3000, 50, "夹爪闭合超时")

    -- 关闭输出
    setDO(GRIPPER_CLOSE_OUT, 0)
    setDO(GRIPPER_OPEN_OUT, 0)

    if not ok then
        setError(ERR_GRIPPER_CLOSE_TIMEOUT)  -- 设置夹爪闭合超时错误
        return
    end

    -- 更新状态
    setRegisterBool(COIL_GRIPPER_OPEN, 0)   -- 夹爪张开反馈
    setRegisterBool(COIL_GRIPPER_CLOSE, 1)  -- 夹爪闭合反馈
    g_cellGrabbed = true                     -- 更新抓取标志
    setRegisterInt(REG_ARM_STAT, 5)          -- 机械臂状态设为抓取中
    print("[夹爪控制] 闭合完成")
end

--[[
函数名：gripperHome
功能：夹爪回零
参数：无
返回：无
]]
function gripperHome()
    print("[夹爪控制] 执行回零动作")

    -- 输出回零信号
    setDO(GRIPPER_CLOSE_OUT, 1)  -- 打开闭合输出
    setDO(GRIPPER_OPEN_OUT, 1)   -- 打开张开输出

    -- 等待回零到位
    local ok = waitWithTimeout(
        function()
            return getDI(GRIPPER_CLOSE_IN) == 1 and getDI(GRIPPER_OPEN_IN) == 1
        end,
        3000, 50, "夹爪回零超时")

    -- 关闭输出
    setDO(GRIPPER_CLOSE_OUT, 0)
    setDO(GRIPPER_OPEN_OUT, 0)

    if not ok then
        setError(ERR_GRIPPER_HOME_TIMEOUT)  -- 设置夹爪回零超时错误
        return
    end

    -- 更新状态
    setRegisterBool(COIL_GRIPPER_OPEN, 0)   -- 夹爪张开反馈
    setRegisterBool(COIL_GRIPPER_CLOSE, 0)  -- 夹爪闭合反馈
    g_cellGrabbed = false                    -- 更新抓取标志
    print("[夹爪控制] 回零完成")
end

-- ===================== 第十部分：安全校验模块 =====================
--[[
函数名：isRobotAtHome
功能：判断机械臂是否在原点附近
参数：tolerance - 单轴允许偏差角度，默认5°
返回：bool true=在原点 false=不在原点
]]
function isRobotAtHome(tolerance)
    tolerance = tolerance or 5
    local currentJoints = getJoint()

    -- 检查关节数据有效性
    if not currentJoints or not currentJoints.jp then
        print("[原点校验] 获取关节角度失败")
        return false
    end

    -- 检查原点位姿定义
    if not pHome or not pHome.jp then
        print("[原点校验] 预定义原点pHome不存在")
        return false
    end

    -- 计算各轴偏差总和
    local totalDiff = 0
    for i = 1, 6 do
        totalDiff = totalDiff + math.abs(currentJoints.jp[i] - pHome.jp[i])
    end

    -- 判断是否在原点附近
    local atHome = totalDiff < tolerance * 6
    if atHome then
        print(string.format("[原点校验] 在原点附近，总偏差:%.2f°", totalDiff))
    else
        print(string.format("[原点校验] 不在原点，总偏差:%.2f°", totalDiff))
    end
    return atHome
end

--[[
函数名：ensureRobotAtHome
功能：确保机械臂回到原点，失败则报错
参数：无
返回：bool true=成功回原 false=回原失败
]]
function ensureRobotAtHome()
    -- 先检查是否已在原点
    if isRobotAtHome() then
        return true
    end

    print("[原点校验] 不在原点，尝试自动回原")

    -- 获取当前TCP位姿
    local Ptemp = getTCP({coor = 0})
    print(string.format("[原点校验] 当前Z高度:%.2f", Ptemp.cp[3]))

    -- 根据高度抬升Z轴确保安全
    if Ptemp.cp[3] < 500 then
        -- 高度较低时抬升300mm
        Ptemp.cp[3] = Ptemp.cp[3] + 300
        print("条件1", "判断后坐标Z轴高度：", Ptemp.cp[3])
        movL(Ptemp)
    elseif Ptemp.cp[3] >= 500 and Ptemp.cp[3] < 650 then
        -- 中间高度直接抬升到800mm
        Ptemp.cp[3] = 800
        print("条件2", "判断后坐标Z轴高度：", Ptemp.cp[3])
        movL(Ptemp)
    elseif (Ptemp.cp[3] >= 650 and g_cellGrabbed == false) or Ptemp.cp[3] > 800 then
        -- 高度安全且无电芯，无需调整
        Ptemp.cp[3] = Ptemp.cp[3]
        print("条件3", "判断后坐标Z轴高度：", Ptemp.cp[3])
    elseif Ptemp.cp[3] >= 650 and Ptemp.cp[3] < 800 and g_cellGrabbed == true then
        -- 有电芯且高度在安全范围，抬升到800mm
        Ptemp.cp[3] = 800
        print("条件4", "判断后坐标Z轴高度：", Ptemp.cp[3])
        movL(Ptemp)
    end

    -- 选择过渡点回原
    if Ptemp.cp[2] >= 0 then
        movJ(GD_ZC)  -- 左侧过渡点
    else
        movJ(GD_YC)  -- 右侧过渡点
    end
    movJ(pHome)      -- 回到工程原点
    wait(100)

    -- 验证回原结果
    if isRobotAtHome() then
        print("[原点校验] 自动回原成功")
        return true
    end

    print("[原点校验] 自动回原失败，需人工干预")
    setError(ERR_SYS_ARM_NOT_HOME)
    return false
end

--[[
函数名：checkMatrixBounds
功能：校验行列层是否在允许范围内
参数：row, col, layer - 行列层号
返回：bool true=合法 false=超限
]]
function checkMatrixBounds(row, col, layer)
    -- 读取实际最大行列层数（单一事实源：行列表寄存器）
    local maxRow, maxCol, maxLayer = getTrayDims()

    -- 校验参数是否超限
    if row < 1 or row > maxRow or col < 1 or col > maxCol
        or layer < 1 or layer > maxLayer then
        print(string.format("[矩阵校验] 参数超限 行:%d 列:%d 层:%d | 上限 行:%d 列:%d 层:%d",
            row, col, layer, maxRow, maxCol, maxLayer))
        return false
    end

    print(string.format("[矩阵校验] 参数合法 行:%d 列:%d 层:%d", row, col, layer))
    return true
end

--[[
函数名：checkPlacePosition
功能：校验放料位置与上位机期望是否一致
参数：
  curCount - 当前电芯序号
  layer    - 当前层号
返回：bool true=一致 false=不一致
]]
function checkPlacePosition(curCount, layer)
    -- 获取当前托盘最大行列数（单一事实源：行列表寄存器）
    local curMaxRow, curMaxCol, _, curTotal = getTrayDims()

    -- 计算实际行列号
    local realIndex = curCount
    if PLACE_REVERSE_ORDER then
        realIndex = curTotal - curCount + 1
    end
    local physRow = math.floor((realIndex - 1) / curMaxCol) + 1
    local physCol = (realIndex - 1) % curMaxCol + 1
    local physLayer = layer

    -- 读取上位机期望位置
    local expRow = getRegisterInt(REG_PLACE_ROW)
    local expCol = getRegisterInt(REG_PLACE_COL)
    local expLayer = getRegisterInt(REG_PLACE_LAYER)
    local needCheck = (expRow ~= 0) or (expCol ~= 0) or (expLayer ~= 0)

    -- 上位机未下发期望位置时跳过校验
    if not needCheck then
        print("[放料校验] 上位机未下发期望位置，跳过校验")
        return true
    end

    -- 逐项校验位置一致性
    local match = true
    if expRow ~= 0 and expRow ~= physRow then match = false end
    if expCol ~= 0 and expCol ~= physCol then match = false end
    if expLayer ~= 0 and expLayer ~= physLayer then match = false end

    if not match then
        print(string.format("[放料校验] 位置不匹配 期望(层:%d行:%d列:%d) 实际(层:%d行:%d列:%d)",
            expLayer, expRow, expCol, physLayer, physRow, physCol))
        setError(ERR_PARAM_PLACE_POS_MISMATCH)
        return false
    end

    print("[放料校验] 位置一致性校验通过")
    return true
end

-- ===================== 第十一部分：系统初始化 =====================
--[[
函数名：robotInit
功能：系统全局初始化
参数：无
返回：bool true=初始化成功 false=初始化失败
]]
function robotInit()
    print("[系统初始化] 开始执行初始化流程")
    clearError()  -- 清除历史错误
    setRegisterInt(REG_ROBOT_STAT, 1)  -- 设置机器人状态为运行中

    -- 设置低速运动参数（初始化阶段使用安全速度）
    setSpeedJ(SPEED_J / 2)
    setAccJ(ACC_J / 2)
    setSpeedL(SPEED_L / 2)
    setAccL(ACC_L / 2)

    -- 机械臂回原点
    ensureRobotAtHome()
    wait(500)

    -- 电芯状态一致性检测
    if ENABLE_CELL_DETECTION then
        local hasCell = getDI(CELL_DETECTION_IO) == 1
        -- 检测传感器与记忆状态是否一致
        if hasCell and not g_cellGrabbed then
            print("[初始化] 状态异常：传感器检测到电芯但无抓取记忆")
            setError(ERR_CELL_MEM_HAS_SENSOR_NO_INIT)
            return false
        elseif not hasCell and g_cellGrabbed then
            print("[初始化] 状态异常：有抓取记忆但传感器未检测到电芯")
            setError(ERR_CELL_MEM_NO_SENSOR_HAS_INIT)
            return false
        end
        g_cellPresent = hasCell
        print(string.format("[初始化] 电芯状态: %s", g_cellPresent and "有料" or "无料"))
    else
        g_cellPresent = g_cellGrabbed
        print("[初始化] 电芯检测禁用，使用软件标志位")
    end

    -- 夹爪回零处理(机器人上电回零判断，之后置位回零标志寄存器)
    if getRegisterBool(COIL_GRIPPER_HOME) == 0 then
        if g_cellPresent == false then
            gripperHome()  -- 无电芯时可安全回零
        elseif g_cellPresent == true then
            print("[初始化] 检测到夹爪有电芯标志位,禁止回零！")
        end
        setRegisterBool(COIL_GRIPPER_HOME, 1)
    end

    -- 恢复正式运动参数
    setSpeedJ(SPEED_J)
    setAccJ(ACC_J)
    setSpeedL(SPEED_L)
    setAccL(ACC_L)

    -- 初始化所有状态寄存器
    setRegisterInt(REG_ARM_STAT, 0)      -- 机械臂状态空闲
    setRegisterInt(REG_ROBOT_STAT, 0)    -- 机器人整体状态空闲
    setRegisterInt(REG_PICK_STAT, 0)     -- 取料状态
    setRegisterInt(REG_PLACE_STAT, 0)    -- 放料状态
    setRegisterInt(REG_CMD_ACK, 0)       -- 指令应答
    setRegisterInt(REG_CMD_CODE, 0)      -- 指令码

    print("[系统初始化] 初始化完成")
    return true
end

-- ===================== 第十二部分：取料核心流程 =====================
--[[
函数名：pickSequence
功能：完整取料动作流程
参数：
  row   - 行号
  col   - 列号
  layer - 层号
返回：无
]]
function pickSequence(row, col, layer)
    print(string.format("\n===== 取料流程启动 第%d层 第%d行 第%d列 =====", layer, row, col))

    -- 保存任务参数
    g_row = row
    g_col = col
    g_layer = layer

    -- 更新状态反馈
    setRegisterInt(REG_CUR_ROW, row)       -- 当前取料行号
    setRegisterInt(REG_CUR_COL, col)       -- 当前取料列号
    setRegisterInt(REG_CUR_LAYER, layer)   -- 当前取料层号
    setRegisterInt(REG_ARM_STAT, 1)        -- 机械臂状态取料中
    setRegisterInt(REG_ROBOT_STAT, 1)      -- 机器人整体状态运行中
    setRegisterInt(REG_CMD_ACK, 1)         -- 指令应答进行中

    -- 前置参数校验
    if not checkParamValid(row, col, layer) then
        return  -- 参数非法，setError已处理
    end
    if not checkMatrixBounds(row, col, layer) then
        setError(ERR_PICK_MATRIX_OVERFLOW)  -- 取料矩阵参数超限
        return
    end

    -- 获取站点侧位（0=左侧，1=右侧）
    local stationSide = getRegisterInt(REG_STATION_SIDE)
    print(string.format("站点侧位：%s", stationSide == 1 and "右侧" or "左侧"))

    -- 选择对应层步长
    local stepX, stepY, stepZ
    if layer == 4 then
        -- 第4层使用专属步长
        stepX = PICK_STEP_X_L4
        stepY = PICK_STEP_Y_L4
        stepZ = PICK_STEP_Z_L4
    else
        -- 1~3层使用标准步长
        stepX = PICK_STEP_X
        stepY = PICK_STEP_Y
        stepZ = PICK_STEP_Z
    end

    -- 获取排列方向
    local retry = 0
    ::GET_MATRIX::
    if retry >= RETRY_MAX then
        print("[取料方向] 获取排列方向重试失败")
        setError(ERR_MATRIX_RETRY_FAIL_PICK)
        return
    end
    retry = retry + 1

    local currentMatrix
    if USE_CAMERA_MATRIX_DETECTION then
        -- 使用相机检测方向
        if row == 1 and col == 1 then
            -- 每层首个电芯触发方向判定
            local dirPose = zShiftPose(stationSide == 0 and pCamera_Dir_ZC or pCamera_Dir_YC)  -- 拍照Z按产品平移
            local dirData = camera_comm(CUR_PRODUCT.camId.dir, dirPose, stationSide)
            if not dirData then
                return  -- 相机拍照失败，setError已处理
            end

            -- 通过高度判定层数（区间来自当前产品参数表）
            local height = dirData[8]
            local detectLayer
            local lh = CUR_PRODUCT.layerHeights
            for i = 1, #lh do
                if height >= lh[i][1] and height <= lh[i][2] then
                    detectLayer = i
                    break
                end
            end
            if not detectLayer then
                print("[取料方向] 高度异常，无法判定层数")
                setError(ERR_CAM_HEIGHT_INVALID_PICK)
                goto GET_MATRIX
            end

            -- 校验层数一致性
            if detectLayer ~= layer then
                print(string.format("[取料方向] 层数不匹配 相机判定:%d 期望:%d", detectLayer, layer))
                setError(ERR_CELL_LAYER_MISMATCH_PICK)
                return
            end

            -- 校验方向值合法性
            local dir = dirData[12]
            if dir ~= 0 and dir ~= 1 then
                print("[取料方向] 方向值非法: " .. dir)
                setError(ERR_CAM_DIR_INVALID_PICK)
                goto GET_MATRIX
            end

            g_pickMatrixForLayer = dir
            print(string.format("[取料方向] 相机判定方向: %d", dir))
        end
        currentMatrix = g_pickMatrixForLayer
    else
        -- 使用上位机下发的方向
        currentMatrix = getRegisterInt(REG_PICK_MATRIX)
    end

    -- 方向缺失防呆（v3.4.6）：无层方向记忆且本点非(1,1)未触发相机判定时，禁止盲目选点
    if currentMatrix == nil then
        print("[取料方向] 无排列方向记忆（请从该层(1,1)重新开始取料）")
        setError(ERR_MATRIX_RETRY_FAIL_PICK)
        return
    end

    -- 启用预设定方向一致性校验
    if MATRIX_CHANGE then
        if layer == 2 or layer == 4 then
            if currentMatrix ~= 1 then
                print(string.format("[取料方向] 横纵方向固定启用，当前横纵方向与预设定方向不一致 当前:%d", currentMatrix))
                setError(ERR_PRESET_DIR_MISMATCH_PICK)
            end
        elseif layer == 1 or layer == 3 then
            if currentMatrix ~= 0 then
                print(string.format("[取料方向] 横纵方向固定启用，当前横纵方向与预设定方向不一致 当前:%d", currentMatrix))
                setError(ERR_PRESET_DIR_MISMATCH_PICK)
            end
        end
    end

    -- 同层方向一致性校验
    if g_lastPickLayer == layer then
        if currentMatrix ~= g_lastPickMatrix then
            print(string.format("[取料方向] 同层方向不一致 上次:%d 当前:%d",
                g_lastPickMatrix, currentMatrix))
            setError(ERR_SAME_LAYER_DIR_MISMATCH_PICK)
        end
    else
        g_lastPickLayer = layer
        g_lastPickMatrix = currentMatrix
    end
    g_cellMatrix = currentMatrix
    print(string.format("[取料方向] 最终排列方向: %d", g_cellMatrix))

    -- 计算拍照位
    local baseCamPose = nil
    local snapPose = nil
    if g_cellMatrix == 0 then
        -- X优先方向
        if stationSide == 0 then
            baseCamPose = layer == 4 and pCamera_Grab_XZ_L4 or pCamera_Grab_XZ
            snapPose = calcPickOffset(row, col, layer, baseCamPose, stepX, stepY, stepZ)
        else
            baseCamPose = layer == 4 and pCamera_Grab_XY_L4 or pCamera_Grab_XY
            snapPose = calcPickOffset(row, col, layer, baseCamPose, stepX, -stepY, stepZ)
        end
    else
        -- Y优先方向
        if stationSide == 0 then
            baseCamPose = layer == 4 and pCamera_Grab_YZ_L4 or pCamera_Grab_YZ
            snapPose = calcPickOffset(row, col, layer, baseCamPose, stepX, stepY, stepZ)
        else
            baseCamPose = layer == 4 and pCamera_Grab_YY_L4 or pCamera_Grab_YY
            snapPose = calcPickOffset(row, col, layer, baseCamPose, -stepX, stepY, stepZ)
        end
    end

    -- 计算当前点相对基准点的理论XY偏移
    local dx_theo_pick = snapPose.cp[1] - baseCamPose.cp[1]
    local dy_theo_pick = snapPose.cp[2] - baseCamPose.cp[2]

    -- 拍照获取坐标
    local camId = layer == 4 and CUR_PRODUCT.camId.grabL4 or CUR_PRODUCT.camId.grab
    local pointArr = camera_comm(camId, snapPose, stationSide)
    if not pointArr then
        return  -- 相机拍照失败，setError已处理
    end
    applyCameraPose(pStation_Grab, pointArr)
    wait(20)

    -- 坐标偏差校验（基于矩阵理论偏移）
    local temPose = {
        cp = {
            pointArr[6], pointArr[7], pointArr[8],
            pointArr[9], pointArr[10], pointArr[11]
        }
    }
    print("temPose: ", temPose)
    checkPickDeviation(layer, temPose, dx_theo_pick, dy_theo_pick, row, col)

    -- 每层最后一个电芯后清除基准（按当前产品规格）
    local maxRowEnd, maxColEnd = getTrayDims()
    if row == maxRowEnd and col == maxColEnd then
        g_pickBasePose[layer] = nil
    end

    -- 误差偏移补偿
    local xOffset, yOffset
    if ERROR_OFFSET_SELECT then
        if g_cellMatrix == 0 then
            if stationSide == 0 then
                xOffset = (col - 1) * (PICK_X_OFFSET / PALLET_COLS)
                yOffset = -(row - 1) * (PICK_Y_OFFSET / PALLET_ROWS)
            else
                xOffset = (col - 1) * (PICK_X_OFFSET / PALLET_COLS)
                yOffset = (row - 1) * (PICK_Y_OFFSET / PALLET_ROWS)
            end
        else
            if stationSide == 0 then
                xOffset = (row - 1) * (PICK_X_OFFSET / PALLET_ROWS)
                yOffset = -(col - 1) * (PICK_Y_OFFSET / PALLET_COLS)
            else
                xOffset = (row - 1) * (PICK_X_OFFSET / PALLET_ROWS)
                yOffset = (col - 1) * (PICK_Y_OFFSET / PALLET_COLS)
            end
        end
    else
        xOffset = PICK_X_OFFSET
        yOffset = PICK_Y_OFFSET
    end

    -- 应用误差补偿和高度补偿
    pStation_Grab.cp[1] = pStation_Grab.cp[1] + xOffset
    pStation_Grab.cp[2] = pStation_Grab.cp[2] + yOffset
    pStation_Grab.cp[3] = pStation_Grab.cp[3] + PICK_HEIGHT_OFFSET
    pStation_Grab.cp[4] = -179.989

    print(string.format("[取料位] 最终坐标 X:%.2f Y:%.2f Z:%.2f RX:%.2f RY:%.2f RZ:%.2f",
        pStation_Grab.cp[1], pStation_Grab.cp[2], pStation_Grab.cp[3], 
        pStation_Grab.cp[4], pStation_Grab.cp[5], pStation_Grab.cp[6]))

    -- 计算安全位与扫码位
    pStation_Grab_Up = coorRel(pStation_Grab, {0, 0, SAFE_HEIGHT, 0, 0, 0})
    pStation_Grab_Scan = coorRel(pStation_Grab, {0, 0, SCAN_HEIGHT, 0, 0, 0})

    -- 移动到扫码位（可选寻位检测）
    if SEARCH_CHANGE then
        movLWithSearch(pStation_Grab_Scan)
    else
        movL(pStation_Grab_Scan)
    end
	print("当前扫码位关节: ",getJoint())
	
    -- 等待扫码完成信号
    local scanRetry = 0
    ::SCAN_WAIT::
    if scanRetry >= RETRY_MAX then
        print("[取料流程] 扫码等待重试失败")
        setError(ERR_SCAN_RETRY_FAIL)
        return
    end
    scanRetry = scanRetry + 1

    setRegisterInt(REG_ARM_STAT, 4)  -- 机械臂状态设为扫码中
    print("[取料流程] 等待上位机扫码完成信号")

    local scanOk = waitWithTimeout(
        function() return getRegisterInt(REG_SCAN_DONE) == 1 end,
        60000, POLL_INTERVAL, "扫码等待超时")

    if not scanOk then
        setError(ERR_SYS_SCAN_TIMEOUT)  -- 扫码超时
        goto SCAN_WAIT
    end

    setRegisterInt(REG_SCAN_DONE, 0)  -- 清除扫码完成标志
    print("[取料流程] 扫码完成")

    -- 下降接近取料位
    local approachPose = coorRel(pStation_Grab, {0, 0, 100, 0, 0, 0})
	local grapperStatus = (getDI(GRIPPER_CLOSE_IN) == 1 and getDI(GRIPPER_OPEN_IN) == 1)
	if not grapperStatus then
		print("[取料流程] 夹爪不在张开状态，有碰撞风险")
		setError(ERR_GRIPPER_NOT_OPEN) 
		return
	end
    if SEARCH_CHANGE then
        movLWithSearch(approachPose)
        movLWithSearch(pStation_Grab, {v = 200, a = 500})
    else
        movL(approachPose)
        movL(pStation_Grab, {v = 200, a = 500})
    end

    -- 电芯存在检测
    if ENABLE_CELL_DETECTION then
        print("[取料流程] 电芯存在检测")
        if getDI(CELL_DETECTION_IO) ~= 1 then
            print("[取料流程] 未检测到电芯")
            setError(ERR_CELL_PICK_EMPTY)
            return
        end
        print("[取料流程] 电芯检测通过")
    end

    -- 取料前测电压（取料位Z方向下移固定距离测量，测完回到抓取位再抓取）
    if ENABLE_VOLTAGE_CHECK then
        local voltagePose = coorRel(pStation_Grab, {0, 0, VOLTAGE_CHECK_OFFSET, 0, 0, 0})
        print(string.format("[测电压] 测电压位 Z偏移:%.1fmm", VOLTAGE_CHECK_OFFSET))

        local voltRetry = 0
        ::VOLTAGE_RETRY::

        if voltRetry >= RETRY_MAX then
            print("[测电压] 测电压等待重试失败")
            setError(ERR_VOLTAGE_RETRY_FAIL)
            return
        end
        voltRetry = voltRetry + 1

        -- 移等待上位机确认是否需要测电压
		local voltsignOK = waitWithTimeout(
			function() return getRegisterInt(REG_VOLTAGE_START) ~= 0 end,
			60000, POLL_INTERVAL, "是否需要测电压信号等待超时")

		if not voltsignOK then
			setError(ERR_VOLTAGE_SIGN_TIMEOUT)  -- 测电压信号等待超时
			goto VOLTAGE_RETRY
		end

        -- 测电压动作流程
		if getRegisterInt(REG_VOLTAGE_START) == 1 then
			-- 移动到测电压位（取料位下移 VOLTAGE_CHECK_OFFSET mm）
			if SEARCH_CHANGE then
				movLWithSearch(voltagePose)
			else
				movL(voltagePose)
			end
			setRegisterInt(REG_VOLTAGE_DONE, 0)  -- 清除测电压完成标志
			wait(100)
			setRegisterInt(REG_ARM_STAT, 6)  -- 机械臂状态设为测电压中
			print("[测电压] 等待上位机测电压完成信号")

			local voltOk = waitWithTimeout(
				function() return getRegisterInt(REG_VOLTAGE_DONE) == 2 end,
				60000, POLL_INTERVAL, "测电压等待超时")

			if not voltOk then
				setError(ERR_VOLTAGE_TIMEOUT)  -- 测电压超时
				setRegisterInt(REG_VOLTAGE_DONE, 0)  -- 清除测电压完成标志
				goto VOLTAGE_RETRY
			end

			setRegisterInt(REG_VOLTAGE_DONE, 0)  -- 清除测电压完成标志
			print("[测电压] 测电压完成")
		end
		setRegisterInt(REG_VOLTAGE_START, 0)  -- 清除测电压需求信号

        -- 测电压完成后回到抓取位
        if SEARCH_CHANGE then
            movLWithSearch(pStation_Grab)
        else
            movL(pStation_Grab)
        end
    end

    -- 闭合夹爪抓取
    gripperClose()
    wait(100)

    -- 抬升到安全位
    movL(pStation_Grab_Up)

    -- 电芯抓取确认检测
    if ENABLE_CELL_DETECTION then
        print("[取料流程] 电芯抓取确认检测")
        if getDI(CELL_DETECTION_IO) ~= 1 then
            print("[取料流程] 未检测到电芯")
            setError(ERR_CELL_GRAB_CONFIRM)
            return
        end
        print("[取料流程] 电芯检测通过")
    end

    -- 机械臂回原点
    local curPose = getTCP({coor = 0, tool = 1})
    if curPose.cp[2] >= 0 then
        movJ(GD_ZC, {b = 100})
        movJ(pHome, {v = 120, a = 200})
    else
        movJ(GD_YC, {b = 100})
        movJ(pHome, {v = 120, a = 200})
    end

    -- 更新任务状态
    setRegisterInt(REG_PICK_STAT, 1)   -- 取料完成
    setRegisterInt(REG_ARM_STAT, 0)    -- 机械臂空闲
    setRegisterInt(REG_ROBOT_STAT, 0)  -- 机器人整体空闲
    setRegisterInt(REG_CMD_ACK, 2)     -- 指令应答完成

    print(string.format("===== 取料流程完成 第%d层 第%d行 第%d列 =====\n", layer, row, col))
end

-- ===================== 第十三部分：放料核心流程 =====================
--[[
函数名：placeSequence
功能：完整放料动作流程
参数：station - 目标站点号
返回：bool true=放料成功 false=放料失败
]]
function placeSequence(station)
    print(string.format("\n===== 放料流程启动 站点:%d =====", station))

    -- 更新状态反馈
    setRegisterInt(REG_ARM_STAT, 2)        -- 机械臂状态放料中
    setRegisterInt(REG_ROBOT_STAT, 1)      -- 机器人整体状态运行中
    setRegisterInt(REG_CMD_ACK, 1)         -- 指令应答执行中

    -- 初始化站点状态
    initStationState(station)
    local st = g_stationState[station]

    -- 前置参数校验
    if not checkParamValid(1, 1, st.layer, station) then
        return false
    end

    -- 盘满检测与换盘（CONFIRM_FULL=1为上位机预确认：物理动作已完成，消费即清零、直接执行不等待）
	local curstation = getRegisterInt(REG_STACK_STATION)
	local fullPreconfirmed = false   -- 进入时已带上位机换层预确认
    if getRegisterInt(REG_CONFIRM_FULL) == 1 then
		if curstation == station and st.layer <= 3 then
			st.palletFull = true
			fullPreconfirmed = true
			setRegisterInt(REG_STACK_STATION, 0)
			setRegisterInt(REG_CONFIRM_FULL, 0)
		elseif curstation ~= station then
			print(string.format("[放料流程] 下发换层站点%d，与当前站点%d不符，不予换层", curstation, station))		
		elseif curstation == station and st.layer > 3 then
			-- 顶层收到换层预确认 = 满垛已移走，直接置垛满复位
			st.stackFull = true
			fullPreconfirmed = true
			setRegisterInt(REG_STACK_STATION, 0)
			setRegisterInt(REG_CONFIRM_FULL, 0)
		end
	else
    _, st.palletFull = getPlaceSide(station)
    end

    if st.palletFull then
        print(string.format("[放料流程] 站点%d托盘已满，请更换空盘", station))
        -- 满层后清除当前层基准
        local base_key = string.format("%d_%d", station, st.layer)
        g_placeBasePose[base_key] = nil
        print(string.format("[放料基准] 已清除站点%d第%d层基准，换盘后将重新标定", station, st.layer))

        -- 自然盘满（非预确认进入）时，等待上位机换盘完成信号
        if not fullPreconfirmed then
            local palletChangeOk = waitWithTimeout(
                function() return getRegisterInt(REG_CONFIRM_FULL) == 1 end,
                300000, POLL_INTERVAL, "等待换盘确认超时")

			if not palletChangeOk then
				setError(ERR_SYS_PALLET_TIMEOUT)  -- 等待换盘完成超时错误
				return
			end
        end
        setRegisterInt(REG_CONFIRM_FULL, 0)
        st.count = 0
        st.layer = st.layer + 1
        st.stackFull = st.layer > MAX_LAYER
        print(string.format("[放料流程] 换层完成 当前层数:%d", st.layer))
    end

    -- 垛满检测与复位（CONFIRM_RESET=1同为预确认：消费即清零，直接复位不等待）
    if getRegisterInt(REG_CONFIRM_RESET) == 1 then
		if curstation == station then
			st.stackFull = true
			setRegisterInt(REG_STACK_STATION, 0)
			setRegisterInt(REG_CONFIRM_RESET, 0)
		else 
			print(string.format("[放料流程] 下发清垛站点%d，与当前站点%d不符，不予清垛", curstation, station))		
		end
    end

    if st.stackFull then
        print(string.format("[放料流程] 站点%d料垛已满，请移走满垛", station))
        setRegisterInt(REG_CONFIRM_RESET, 0)

        -- 满垛复位后清除该站点所有层基准
        for key, _ in pairs(g_placeBasePose) do
            if string.match(key, "^" .. tostring(station) .. "_") then
                g_placeBasePose[key] = nil
            end
        end
        print(string.format("[放料基准] 已清除站点%d所有层基准，新料垛将重新标定", station))

        -- 重置站点状态
        g_stationState[station] = {
            stationNum = station,
            count = 0,
            side = 0,
            layer = 1,
            cellMatrix = nil,
            palletFull = false,
            stackFull = false,
            lastMatrix = nil,
            lastLayer = nil
        }
        st = g_stationState[station]
        print("[放料流程] 料垛复位完成")
    end

    -- 当前电芯序号（站点自管理）
    local curCount = st.count + 1
    print(string.format("[放料流程] 第%d层 第%d个电芯", st.layer, curCount))

    -- 前置位置校验（按当前产品规格：行列表寄存器）
    local _, preMaxCol, _, preTotal = getTrayDims()
    local realIndex = curCount
    if PLACE_REVERSE_ORDER then
        realIndex = preTotal - curCount + 1
    end
    local physRow = math.floor((realIndex - 1) / preMaxCol) + 1
    local physCol = (realIndex - 1) % preMaxCol + 1
    local physLayer = st.layer
    if not checkMatrixBounds(physRow, physCol, physLayer) then
        setError(ERR_PLACE_MEM_MATRIX_OVERFLOW)
        return false
    end
    print(string.format("[安全校验] 当前站点记忆要放的行列层：当前行=%d，当前列=%d，当前层=%d", 
        physRow, physCol, physLayer))

    -- 获取站点侧位
    local stationSide = getRegisterInt(REG_STATION_SIDE)
    print(string.format("站点侧位：%s", stationSide == 1 and "右侧" or "左侧"))

    -- 选择对应层步长
    local stepX, stepY, stepZ
    if st.layer == 4 then
        stepX = PLACE_STEP_X_L4
        stepY = PLACE_STEP_Y_L4
        stepZ = PLACE_STEP_Z_L4
    else
        stepX = PLACE_STEP_X
        stepY = PLACE_STEP_Y
        stepZ = PLACE_STEP_Z
    end

    -- 获取排列方向
    local retry = 0
    ::GET_PLACE_MATRIX::
    if retry >= RETRY_MAX then
        print("[放料方向] 获取排列方向重试失败")
        setError(ERR_MATRIX_RETRY_FAIL_PLACE)
        return false
    end
    retry = retry + 1

    local currentMatrix
    if USE_CAMERA_MATRIX_DETECTION then
        -- 每层首个电芯触发方向判定
        if st.count == 0 then
            local dirPose = zShiftPose(stationSide == 0 and pCamera_Dir_ZC or pCamera_Dir_YC)  -- 拍照Z按产品平移
            local dirData = camera_comm(CUR_PRODUCT.camId.dir, dirPose, stationSide)
            if not dirData then
                return false  -- 相机拍照失败，setError已处理
            end

            -- 高度判断层数（区间来自当前产品参数表）
            local height = dirData[8]
            local detectLayer
            local lh = CUR_PRODUCT.layerHeights
            for i = 1, #lh do
                if height >= lh[i][1] and height <= lh[i][2] then
                    detectLayer = i
                    break
                end
            end
            if not detectLayer then
                print("[放料方向] 高度异常，无法判定层数")
                setError(ERR_CAM_HEIGHT_INVALID_PLACE)
                goto GET_PLACE_MATRIX
            end

            -- 校验层数一致性
            if detectLayer ~= st.layer then
                print(string.format("[放料方向] 层数不匹配 相机判定:%d 记忆:%d", detectLayer, st.layer))
                setError(ERR_CELL_LAYER_MISMATCH_PLACE)
                return false
            end

            -- 校验方向值合法性
            local dir = dirData[12]
            if dir ~= 0 and dir ~= 1 then
                print("[放料方向] 方向值非法: " .. dir)
                setError(ERR_CAM_DIR_INVALID_PLACE)
                goto GET_PLACE_MATRIX
            end

            st.cellMatrix = dir
            print(string.format("[放料方向] 相机判定方向: %d", dir))
        end
        currentMatrix = st.cellMatrix
    else
        currentMatrix = getRegisterInt(REG_PLACE_MATRIX)
    end

    -- 方向缺失防呆（v3.4.6）：站点无方向记忆且本电芯未触发相机判定时，禁止盲目选点
    if currentMatrix == nil then
        print("[放料方向] 无排列方向记忆（请从该层第1个电芯重新开始放料）")
        setError(ERR_MATRIX_RETRY_FAIL_PLACE)
        return false
    end

    -- 启用预设定方向一致性校验
    if MATRIX_CHANGE then
        if st.layer == 2 or st.layer == 4 then
            if currentMatrix ~= 1 then
                print(string.format("[放料方向] 横纵方向固定启用，当前横纵方向与预设定方向不一致 当前:%d", currentMatrix))
                setError(ERR_PRESET_DIR_MISMATCH_PLACE)
            end
        elseif st.layer == 1 or st.layer == 3 then
            if currentMatrix ~= 0 then
                print(string.format("[放料方向] 横纵方向固定启用，当前横纵方向与预设定方向不一致 当前:%d", currentMatrix))
                setError(ERR_PRESET_DIR_MISMATCH_PLACE)
            end
        end
    end

    -- 同层方向一致性校验
    if st.lastLayer == st.layer then
        if currentMatrix ~= st.lastMatrix then
            print(string.format("[放料方向] 同层方向不一致 上次:%d 当前:%d",
                st.lastMatrix, currentMatrix))
            setError(ERR_SAME_LAYER_DIR_MISMATCH_PLACE)  -- 修正：原误用相机通讯错误码，改为同层方向不一致错误码
        end
    else
        st.lastLayer = st.layer
        st.lastMatrix = currentMatrix
    end
    st.cellMatrix = currentMatrix
    print(string.format("[放料方向] 最终排列方向: %d", st.cellMatrix))

    -- 计算拍照位
    local baseCamPose = nil
    local snapPose = nil
    if st.cellMatrix == 0 then
        if stationSide == 0 then
            baseCamPose = st.layer == 4 and pCamera_Place_XZ_L4 or pCamera_Place_XZ
            snapPose = calcPlaceOffset(st.cellMatrix, curCount, st.layer, baseCamPose, stepX, stepY, stepZ)
        else
            baseCamPose = st.layer == 4 and pCamera_Place_XY_L4 or pCamera_Place_XY
            snapPose = calcPlaceOffset(st.cellMatrix, curCount, st.layer, baseCamPose, stepX, -stepY, stepZ)
        end
    else
        if stationSide == 0 then
            baseCamPose = st.layer == 4 and pCamera_Place_YZ_L4 or pCamera_Place_YZ
            snapPose = calcPlaceOffset(st.cellMatrix, curCount, st.layer, baseCamPose, stepX, stepY, stepZ)
        else
            baseCamPose = st.layer == 4 and pCamera_Place_YY_L4 or pCamera_Place_YY
            snapPose = calcPlaceOffset(st.cellMatrix, curCount, st.layer, baseCamPose, -stepX, stepY, stepZ)
        end
    end

    -- 计算当前点相对基准点的理论XY偏移
    local dx_theo_place = snapPose.cp[1] - baseCamPose.cp[1]
    local dy_theo_place = snapPose.cp[2] - baseCamPose.cp[2]

    -- 拍照获取坐标
    local camId = st.layer == 4 and CUR_PRODUCT.camId.placeL4 or CUR_PRODUCT.camId.place
    local pointArr = camera_comm(camId, snapPose, stationSide)
    if not pointArr then
        return false  -- 相机拍照失败，setError已处理
    end
    applyCameraPose(pStation_Place, pointArr)
    wait(20)

    -- 坐标偏差校验（基于矩阵理论偏移）
    local temPose = {
        cp = {
            pointArr[6], pointArr[7], pointArr[8],
            pointArr[9], pointArr[10], pointArr[11]
        }
    }
    print("temPose: ", temPose)
    checkPlaceDeviation(station, st.layer, temPose, dx_theo_place, dy_theo_place, curCount)

    -- 误差偏移补偿
    local xOffset, yOffset
    if ERROR_OFFSET_SELECT then
        if st.cellMatrix == 0 then
            if stationSide == 0 then
                xOffset = (g_PlaceCol - 1) * (PLACE_X_OFFSET / PALLET_COLS)
                yOffset = -(g_PlaceRow - 1) * (PLACE_Y_OFFSET / PALLET_ROWS)
            else
                xOffset = (g_PlaceCol - 1) * (PLACE_X_OFFSET / PALLET_COLS)
                yOffset = (g_PlaceRow - 1) * (PLACE_Y_OFFSET / PALLET_ROWS)
            end
        else
            if stationSide == 0 then
                xOffset = (g_PlaceRow - 1) * (PLACE_X_OFFSET / PALLET_ROWS)
                yOffset = -(g_PlaceCol - 1) * (PLACE_Y_OFFSET / PALLET_COLS)
            else
                xOffset = (g_PlaceRow - 1) * (PLACE_X_OFFSET / PALLET_ROWS)
                yOffset = (g_PlaceCol - 1) * (PLACE_Y_OFFSET / PALLET_COLS)
            end
        end
    else
        xOffset = PLACE_X_OFFSET
        yOffset = PLACE_Y_OFFSET
    end

    -- 应用误差补偿和高度补偿
    pStation_Place.cp[1] = pStation_Place.cp[1] + xOffset
    pStation_Place.cp[2] = pStation_Place.cp[2] + yOffset
    pStation_Place.cp[3] = pStation_Place.cp[3] + PLACE_HEIGHT_OFFSET

    print(string.format("[放料位] 最终坐标 X:%.2f Y:%.2f Z:%.2f RX:%.2f RY:%.2f RZ:%.2f",
        pStation_Place.cp[1], pStation_Place.cp[2], pStation_Place.cp[3], 
        pStation_Place.cp[4], pStation_Place.cp[5], pStation_Place.cp[6]))

    -- 计算安全位
    pStation_Place_Up = coorRel(pStation_Place, {0, 0, SAFE_HEIGHT, 0, 0, 0})
    wait(10)

    -- 移动到安全位（可选寻位检测）
    if SEARCH_CHANGE then
        movLWithSearch(pStation_Place_Up)
    else
        movL(pStation_Place_Up)
    end
	print("当前安全位关节: ",getJoint())
    -- 放料位空穴检测
    if ENABLE_CELL_DETECTION then
        print("[放料流程] 空穴电芯存在检测")
        if getDI(CELL_DETECTION_IO_1) ~= 0 then
            print("[放料流程] 检测到目标位置已有电芯，禁止放料")
            setError(ERR_CELL_PLACE_OCCUPIED)
            return false
        end
        print("[放料流程] 空穴检测通过")
    end

    -- 下降接近放料位
    local approachPose = coorRel(pStation_Place, {0, 0, 100, 0, 0, 0})

    if SEARCH_CHANGE then
        movLWithSearch(approachPose)
        movLWithSearch(pStation_Place, {v = 200, a = 500})
    else
        movL(approachPose)
        movL(pStation_Place, {v = 200, a = 500})
    end

    -- 张开夹爪放料
    gripperOpen()
    wait(20)

    -- 更新站点计数
    st.count = curCount - 1
    incStationCount(station)
    print(string.format("===== 放料计数完成 站点:%d 第%d层 第%d个 =====\n", station, st.layer, curCount))

    -- 抬升到安全位
    movL(pStation_Place_Up)

    -- 放料后夹爪残留检测
    if ENABLE_CELL_DETECTION then
        print("[放料流程] 夹爪电芯残留检测")
        if getDI(CELL_DETECTION_IO) ~= 0 then
            print("[放料流程] 夹爪仍检测到电芯，放料未成功")
            setError(ERR_CELL_PLACE_RESIDUAL)
            return false
        end
        print("[放料流程] 放料成功，夹爪无残留")
    end

    -- 机械臂回原点
    local curPose = getTCP({coor = 0, tool = 1})
    if curPose.cp[2] >= 0 then
        movJ(GD_ZC, {b = 100})
        movJ(pHome, {v = 120, a = 200})
    else
        movJ(GD_YC, {b = 100})
        movJ(pHome, {v = 120, a = 200})
    end

    -- 更新任务状态
    setRegisterInt(REG_PLACE_STAT, 1)   -- 放料完成
    setRegisterInt(REG_ARM_STAT, 0)     -- 机械臂空闲
    setRegisterInt(REG_ROBOT_STAT, 0)   -- 机器人整体空闲
    setRegisterInt(REG_CMD_ACK, 2)      -- 指令应答完成

    print(string.format("===== 放料流程完成 站点:%d 第%d层 第%d个 =====\n", station, st.layer, curCount))
    return true
end

--[[
函数名：placeSequenceHost
功能：上位机计数驱动的放料动作流程。由上位机下发目标站点、层号和放料计数，
      行列号由计数反推，不依赖站点自管理状态，也不含换层/清垛动作。
参数：
  station     - 目标站点号（REG_STATION）
  layer       - 上位机下发层号（REG_PLACE_LAYER）
  placeCount  - 上位机下发放料计数（REG_PLACE_COUNT）
返回：bool true=放料成功 false=放料失败
]]
function placeSequenceHost(station, layer, placeCount)
    print(string.format("\n===== 上位机计数放料流程启动 站点:%d 第%d层 第%d个 =====", station, layer, placeCount))

    -- 更新状态反馈
    setRegisterInt(REG_ARM_STAT, 2)        -- 机械人状态放料中
    setRegisterInt(REG_ROBOT_STAT, 1)      -- 机器人整体状态运行中
    setRegisterInt(REG_CMD_ACK, 1)         -- 指令应答进行中

    -- 前置参数校验
    if not checkParamValid(1, 1, layer, station) then
        return false
    end

    -- 当前放料计数（上位机下发）
    local curCount = placeCount
    print(string.format("[放料流程] 第%d层 第%d个电芯", layer, curCount))

    -- 前置位置校验：行列由计数反推（按当前产品规格：行列表寄存器）
    local _, preMaxCol, _, preTotal = getTrayDims()
    local realIndex = curCount
    if PLACE_REVERSE_ORDER then
        realIndex = preTotal - curCount + 1
    end
    local physRow = math.floor((realIndex - 1) / preMaxCol) + 1
    local physCol = (realIndex - 1) % preMaxCol + 1
    local physLayer = layer
    if not checkMatrixBounds(physRow, physCol, physLayer) then
        setError(ERR_PLACE_UPLOAD_MATRIX_OVERFLOW)
        return false
    end
    print(string.format("[安全校验] 上位机计数反推行列层：当前行=%d，当前列=%d，当前层=%d",
        physRow, physCol, physLayer))

    -- 初始化站点状态
    initStationState(station)
    local st = g_stationState[station]

    -- 获取站点侧位
    local stationSide = getRegisterInt(REG_STATION_SIDE)
    print(string.format("站点侧位：%s", stationSide == 1 and "右侧" or "左侧"))

    -- 选择对应层步长
    local stepX, stepY, stepZ
    if layer == 4 then
        stepX = PLACE_STEP_X_L4
        stepY = PLACE_STEP_Y_L4
        stepZ = PLACE_STEP_Z_L4
    else
        stepX = PLACE_STEP_X
        stepY = PLACE_STEP_Y
        stepZ = PLACE_STEP_Z
    end

    -- 获取排列方向
    local retry = 0
    ::GET_PLACE_MATRIX::
    if retry >= RETRY_MAX then
        print("[放料方向] 获取排列方向重试失败")
        setError(ERR_MATRIX_RETRY_FAIL_PLACE)
        return false
    end
    retry = retry + 1

    local currentMatrix
    if USE_CAMERA_MATRIX_DETECTION then
        -- 每层首个电芯触发方向判定
        if curCount == 1 then
            local dirPose = zShiftPose(stationSide == 0 and pCamera_Dir_ZC or pCamera_Dir_YC)  -- 拍照Z按产品平移
            local dirData = camera_comm(CUR_PRODUCT.camId.dir, dirPose, stationSide)
            if not dirData then
                return false  -- 相机拍照失败，setError已处理
            end

            -- 高度判断层数（区间来自当前产品参数表）
            local height = dirData[8]
            local detectLayer
            local lh = CUR_PRODUCT.layerHeights
            for i = 1, #lh do
                if height >= lh[i][1] and height <= lh[i][2] then
                    detectLayer = i
                    break
                end
            end
            if not detectLayer then
                print("[放料方向] 高度异常，无法判定层数")
                setError(ERR_CAM_HEIGHT_INVALID_PLACE)
                goto GET_PLACE_MATRIX
            end

            -- 校验层数一致性
            if detectLayer ~= layer then
                print(string.format("[放料方向] 层数不匹配 相机判定:%d 期望:%d", detectLayer, layer))
                setError(ERR_CELL_LAYER_MISMATCH_PLACE)
                return false
            end

            -- 校验方向值合法性
            local dir = dirData[12]
            if dir ~= 0 and dir ~= 1 then
                print("[放料方向] 方向值非法: " .. dir)
                setError(ERR_CAM_DIR_INVALID_PLACE)
                goto GET_PLACE_MATRIX
            end

            st.cellMatrix = dir
            print(string.format("[放料方向] 相机判定方向: %d", dir))
        end
        currentMatrix = st.cellMatrix
    else
        currentMatrix = getRegisterInt(REG_PLACE_MATRIX)
    end

    -- 方向缺失防呆（v3.4.6）：站点无方向记忆且本电芯未触发相机判定时，禁止盲目选点
    if currentMatrix == nil then
        print("[放料方向] 无排列方向记忆（请从该层第1个电芯重新开始放料）")
        setError(ERR_MATRIX_RETRY_FAIL_PLACE)
        return false
    end

    -- 启用预设定方向一致性校验
    if MATRIX_CHANGE then
        if layer == 2 or layer == 4 then
            if currentMatrix ~= 1 then
                print(string.format("[放料方向] 横纵方向固定启用，当前横纵方向与预设定方向不一致 当前:%d", currentMatrix))
                setError(ERR_PRESET_DIR_MISMATCH_PLACE)
            end
        elseif layer == 1 or layer == 3 then
            if currentMatrix ~= 0 then
                print(string.format("[放料方向] 横纵方向固定启用，当前横纵方向与预设定方向不一致 当前:%d", currentMatrix))
                setError(ERR_PRESET_DIR_MISMATCH_PLACE)
            end
        end
    end

    -- 同层方向一致性校验
    if st.lastLayer == layer then
        if currentMatrix ~= st.lastMatrix then
            print(string.format("[放料方向] 同层方向不一致 上次:%d 当前:%d",
                st.lastMatrix, currentMatrix))
            setError(ERR_SAME_LAYER_DIR_MISMATCH_PLACE)
        end
    else
        st.lastLayer = layer
        st.lastMatrix = currentMatrix
    end
    st.cellMatrix = currentMatrix
    print(string.format("[放料方向] 最终排列方向: %d", st.cellMatrix))

    -- 计算拍照位
    local baseCamPose = nil
    local snapPose = nil
    if st.cellMatrix == 0 then
        if stationSide == 0 then
            baseCamPose = layer == 4 and pCamera_Place_XZ_L4 or pCamera_Place_XZ
            snapPose = calcPlaceOffset(st.cellMatrix, curCount, layer, baseCamPose, stepX, stepY, stepZ)
        else
            baseCamPose = layer == 4 and pCamera_Place_XY_L4 or pCamera_Place_XY
            snapPose = calcPlaceOffset(st.cellMatrix, curCount, layer, baseCamPose, stepX, -stepY, stepZ)
        end
    else
        if stationSide == 0 then
            baseCamPose = layer == 4 and pCamera_Place_YZ_L4 or pCamera_Place_YZ
            snapPose = calcPlaceOffset(st.cellMatrix, curCount, layer, baseCamPose, stepX, stepY, stepZ)
        else
            baseCamPose = layer == 4 and pCamera_Place_YY_L4 or pCamera_Place_YY
            snapPose = calcPlaceOffset(st.cellMatrix, curCount, layer, baseCamPose, -stepX, stepY, stepZ)
        end
    end

    -- 计算当前点相对基准点的理论XY偏移
    local dx_theo_place = snapPose.cp[1] - baseCamPose.cp[1]
    local dy_theo_place = snapPose.cp[2] - baseCamPose.cp[2]

    -- 拍照获取坐标
    local camId = layer == 4 and CUR_PRODUCT.camId.placeL4 or CUR_PRODUCT.camId.place
    local pointArr = camera_comm(camId, snapPose, stationSide)
    if not pointArr then
        return false  -- 相机拍照失败，setError已处理
    end
    applyCameraPose(pStation_Place, pointArr)
    wait(20)

    -- 坐标偏差校验（基于矩阵理论偏移）
    local temPose = {
        cp = {
            pointArr[6], pointArr[7], pointArr[8],
            pointArr[9], pointArr[10], pointArr[11]
        }
    }
    print("temPose: ", temPose)

    -- 每层首个电芯时清除旧基准，让 checkPlaceDeviation 重新记录
    if curCount == 1 then
        local base_key = string.format("%d_%d", station, layer)
        g_placeBasePose[base_key] = nil
        print(string.format("[放料基准] 上位机模式：清除站点%d第%d层旧基准，准备重新标定", station, layer))
    end
    checkPlaceDeviation(station, layer, temPose, dx_theo_place, dy_theo_place, curCount)

    -- 误差偏移补偿
    local xOffset, yOffset
    if ERROR_OFFSET_SELECT then
        if st.cellMatrix == 0 then
            if stationSide == 0 then
                xOffset = (g_PlaceCol - 1) * (PLACE_X_OFFSET / PALLET_COLS)
                yOffset = -(g_PlaceRow - 1) * (PLACE_Y_OFFSET / PALLET_ROWS)
            else
                xOffset = (g_PlaceCol - 1) * (PLACE_X_OFFSET / PALLET_COLS)
                yOffset = (g_PlaceRow - 1) * (PLACE_Y_OFFSET / PALLET_ROWS)
            end
        else
            if stationSide == 0 then
                xOffset = (g_PlaceRow - 1) * (PLACE_X_OFFSET / PALLET_ROWS)
                yOffset = -(g_PlaceCol - 1) * (PLACE_Y_OFFSET / PALLET_COLS)
            else
                xOffset = (g_PlaceRow - 1) * (PLACE_X_OFFSET / PALLET_ROWS)
                yOffset = (g_PlaceCol - 1) * (PLACE_Y_OFFSET / PALLET_COLS)
            end
        end
    else
        xOffset = PLACE_X_OFFSET
        yOffset = PLACE_Y_OFFSET
    end

    -- 应用误差补偿和高度补偿
    pStation_Place.cp[1] = pStation_Place.cp[1] + xOffset
    pStation_Place.cp[2] = pStation_Place.cp[2] + yOffset
    pStation_Place.cp[3] = pStation_Place.cp[3] + PLACE_HEIGHT_OFFSET

    print(string.format("[放料位] 最终坐标 X:%.2f Y:%.2f Z:%.2f RX:%.2f RY:%.2f RZ:%.2f",
        pStation_Place.cp[1], pStation_Place.cp[2], pStation_Place.cp[3],
        pStation_Place.cp[4], pStation_Place.cp[5], pStation_Place.cp[6]))

    -- 计算安全位
    pStation_Place_Up = coorRel(pStation_Place, {0, 0, SAFE_HEIGHT, 0, 0, 0})
    wait(10)

    -- 移动到安全位（可选寻位检测）
    if SEARCH_CHANGE then
        movLWithSearch(pStation_Place_Up)
    else
        movL(pStation_Place_Up)
    end
	print("当前安全位关节: ",getJoint())
    -- 放料位空穴检测
    if ENABLE_CELL_DETECTION then
        print("[放料流程] 空穴电芯存在检测")
        if getDI(CELL_DETECTION_IO_1) ~= 0 then
            print("[放料流程] 检测到目标位置已有电芯，禁止放料")
            setError(ERR_CELL_PLACE_OCCUPIED)
            return false
        end
        print("[放料流程] 空穴检测通过")
    end

    -- 下降接近放料位
    local approachPose = coorRel(pStation_Place, {0, 0, 100, 0, 0, 0})

    if SEARCH_CHANGE then
        movLWithSearch(approachPose)
        movLWithSearch(pStation_Place, {v = 200, a = 500})
    else
        movL(approachPose)
        movL(pStation_Place, {v = 200, a = 500})
    end

    -- 张开夹爪放料
    gripperOpen()
    wait(20)

    print(string.format("===== 放料计数完成 站点:%d 第%d层 第%d个 =====\n", station, layer, curCount))
	
    -- 抬升到安全位
    movL(pStation_Place_Up)

    -- 放料后夹爪残留检测
    if ENABLE_CELL_DETECTION then
        print("[放料流程] 夹爪电芯残留检测")
        if getDI(CELL_DETECTION_IO) ~= 0 then
            print("[放料流程] 夹爪仍检测到电芯，放料未成功")
            setError(ERR_CELL_PLACE_RESIDUAL)
            return false
        end
        print("[放料流程] 放料成功，夹爪无残留")
    end

    -- 机械臂回原点
    local curPose = getTCP({coor = 0, tool = 1})
    if curPose.cp[2] >= 0 then
        movJ(GD_ZC, {b = 100})
        movJ(pHome, {v = 120, a = 200})
    else
        movJ(GD_YC, {b = 100})
        movJ(pHome, {v = 120, a = 200})
    end

    -- 更新任务状态
    setRegisterInt(REG_PLACE_STAT, 1)   -- 放料完成
    setRegisterInt(REG_ARM_STAT, 0)     -- 机械臂空闲
    setRegisterInt(REG_ROBOT_STAT, 0)   -- 机器人整体空闲
    setRegisterInt(REG_CMD_ACK, 2)      -- 指令应答完成

    st.count = curCount
    st.layer = layer

    print(string.format("===== 上位机计数放料流程完成 站点:%d 第%d层 第%d个 =====\n", station, layer, curCount))
    return true
end

-- ===================== 第十四部分：主循环调度 =====================
--[[
函数名：mainLoop
功能：程序主循环，轮询上位机指令并调度对应执行流程
参数：无
返回：无
]]
function mainLoop()
    -- 加载附属功能脚本
    if SEARCH_CHANGE then
        print("防碰撞线程屏蔽，使用寻位指令进行柔性拔高。")
    else
        runScript("防碰撞")
    end
    runScript("负载切换")

    -- 初始产品加载：优先使用断电保持的产品号（续跑），无效则默认产品1
    local initProdId = getRegisterInt(REG_PRODUCT_ID)
    if initProdId == 0 or not PRODUCTS[initProdId] then
        initProdId = 1
    end
    loadProduct(initProdId)

    -- 初始化相机通讯
    initCameraSocket()

    -- 设置全局运动参数
    setSpeedJ(SPEED_J)
    setAccJ(ACC_J)
    setSpeedL(SPEED_L)
    setAccL(ACC_L)

    -- 确保机械臂处于原点
    ensureRobotAtHome()

    print("\n========== 电芯取放料程序正式启动 ==========")
    print("等待上位机指令：")
    print("  寄存器 49108 写 1 → 执行取料")
    print("  寄存器 49108 写 2 → 执行放料")
    print(string.format("电芯检测功能：%s", ENABLE_CELL_DETECTION and "启用" or "禁用"))
    print(string.format("相机方向判定：%s", USE_CAMERA_MATRIX_DETECTION and "启用" or "禁用"))
    print(string.format("矩阵误差补偿：%s", ERROR_OFFSET_SELECT and "启用" or "禁用"))
    print(string.format("测电压功能：%s", ENABLE_VOLTAGE_CHECK and "启用" or "禁用"))
    print("============================================\n")

    -- 主循环轮询
    while true do
        -- 产品号切换检测（仅指令间隙执行：动作中变号顺延至当前指令完成后生效）
        local newProdId = getRegisterInt(REG_PRODUCT_ID)
        if newProdId ~= 0 and newProdId ~= CUR_PRODUCT_ID then
            while not loadProduct(newProdId) do
                setError(ERR_PRODUCT_UNDEFINED)  -- 暂停等待人工/上位机修正产品号
                newProdId = getRegisterInt(REG_PRODUCT_ID)
                if newProdId == 0 then break end  -- 清0=放弃切换，沿用当前产品
            end
        end

        -- 读取上位机指令码
        g_cmdCode = getRegisterInt(REG_CMD_CODE)

        -- 实时电芯状态一致性校验
        if ENABLE_CELL_DETECTION then
            local hasCell = getDI(CELL_DETECTION_IO) == 1
            if hasCell and not g_cellGrabbed then
                print("[主循环] 状态异常：传感器检测到电芯但无抓取记忆")
                setError(ERR_CELL_MEM_HAS_SENSOR_NO_RUN)
                return
            elseif not hasCell and g_cellGrabbed then
                print("[主循环] 状态异常：有抓取记忆但传感器未检测到电芯")
                setError(ERR_CELL_MEM_NO_SENSOR_HAS_RUN)
                return
            end
            g_cellPresent = hasCell
        else
            g_cellPresent = g_cellGrabbed
        end

        -- 指令分发处理
        if g_cmdCode == 1 then
            -- 取料指令
            setRegisterInt(REG_PICK_STAT, 0)
            setRegisterInt(REG_PLACE_STAT, 0)
            print("[主循环] 接收到取料指令")

            -- 取料前置状态校验
            if g_cellPresent then
                print("[主循环] 取料失败：夹爪已有电芯，禁止重复取料")
                setError(ERR_CELL_ALREADY)
            else
                -- 读取行列层参数
                local row = getRegisterInt(REG_ROW)
                local col = getRegisterInt(REG_COL)
                local layer = getRegisterInt(REG_LAYER)

                -- 参数默认值处理
                row = row == 0 and 1 or row
                col = col == 0 and 1 or col
                layer = layer == 0 and 1 or layer
				-- 夹爪张开状态检测
				local grapperStatus = (getDI(GRIPPER_CLOSE_IN) == 1 and getDI(GRIPPER_OPEN_IN) == 1)
				if not grapperStatus then
					print("[取料流程] 夹爪不在张开状态，有碰撞风险")
					setError(ERR_GRIPPER_NOT_OPEN) 
					return
				end
                -- 执行取料流程
                pickSequence(row, col, layer)
            end

            -- 执行完成后清空指令
            setRegisterInt(REG_CMD_CODE, 0)

        elseif g_cmdCode == 2 then
            -- 放料指令
            setRegisterInt(REG_PICK_STAT, 0)
            setRegisterInt(REG_PLACE_STAT, 0)
            print("[主循环] 接收到放料指令")

            -- 放料前置状态校验
            if not g_cellPresent then
                print("[主循环] 放料失败：夹爪无电芯，无法放料")
                setError(ERR_CELL_UNREADY)
            else
                -- 读取目标站点
                local station = getRegisterInt(REG_STATION)
                station = station == 0 and 1 or station

                -- 依据切换开关选择放料模式：上位机动态信号 COIL_CONFIRM_2 优先，常量 USE_HOST_PLACE_MODEL 做默认
                if getRegisterBool(COIL_CONFIRM_2) == 1 or USE_HOST_PLACE_MODEL then
                    local layer = getRegisterInt(REG_PLACE_LAYER)
                    local placeCount = getRegisterInt(REG_PLACE_COUNT)
                    placeSequenceHost(station, layer, placeCount)
                else
                    placeSequence(station)
                end
            end

            -- 执行完成后清空指令
            setRegisterInt(REG_CMD_CODE, 0)

        else
            -- 无指令，维持空闲状态
            setRegisterInt(REG_ARM_STAT, 0)
            setRegisterInt(REG_ROBOT_STAT, 0)
            wait(POLL_INTERVAL)
        end
    end
end

-- ===================== 第十五部分：程序入口 =====================
-- 系统初始化成功后进入主循环，失败则终止程序
if robotInit() then
    mainLoop()
else
    print("[程序入口] 系统初始化失败，程序终止运行")
    stopProject()
end