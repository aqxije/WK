--Lua version 5.3 防碰撞线程（修复版 v1.1）
-- 仅 SEARCH_CHANGE == false 时由主程序 runScript("防碰撞") 加载
-- DI10 触发 → 报错1001并暂停；恢复后状态寄存器还原
while true do
    if getDI(10) == 1 then
		print("寻位信号触发暂停：","getDI(10) == 1")
        -- 缓存各状态地址当前状态值
        local armStatus = getRegisterInt("49012")
        local robotStatus = getRegisterInt("50014")
        local cmdAck = getRegisterInt("49128")
    	-- 置位各状态地址错误状态值
        setRegisterInt("50008", 1001)     -- 写入错误码
        setRegisterInt("49012", 3)             -- 机械臂状态设为错误
        setRegisterInt("50014", 3)           -- 机器人整体状态设为错误
        setRegisterInt("49128", -1001)       -- 指令应答返回负错误码
		-- 暂停程序，等待排查
        pauseProject()
		-- 暂停复位继续，则恢复各状态地址状态值
		setRegisterInt("50008", 0)   -- 清除错误码
		setRegisterInt("49012", armStatus)        -- 机械臂状态恢复
		setRegisterInt("50014", robotStatus)    -- 机器人整体状态恢复
		setRegisterInt("49128", cmdAck)            -- 指令应答恢复
    end
    wait(50)
end
