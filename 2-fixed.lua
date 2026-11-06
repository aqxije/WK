--Lua version 5.3 负载切换线程（修复版 v1.3）
-- 速度倍率来源：REG_MOVE_RATE(42001)，库卓系统速度倍率寄存器，上位机下发，协议范围0~100
-- 程序内防呆限幅：有效速度倍率限制在1~90（91~100自动限幅为90并打印提示；0或>100视为无效）
-- 抓料用 curRate，空手用 curRate+10（同样封顶90）
-- 42001 为 0 或 >100：负载切换不动作；持续1秒 → 错误码1010暂停，恢复后继续检测
local badCnt = 0    -- 无效倍率连续计数（线程局部，跨循环保持）
local lastRaw = nil -- 上次读取的原始倍率（限幅提示去重，避免重复刷屏）
while true do
	local r = getRegisterInt("42001")
	local curRate = 0
	if r > 0 and r <= 100 then
		curRate = r
		if curRate > 90 then
			curRate = 90                     -- 程序内限幅：速度上限90
			if r ~= lastRaw then
				print("[负载切换] 速度倍率", r, "超过90，已限幅为90")
			end
		end
	end
	lastRaw = r
	wait(50)
	-- 速度倍率无效检测：持续1秒（约20个周期）则暂停
	if curRate == 0 then
		badCnt = badCnt + 1
		if badCnt >= 20 then
			print("[负载切换] 速度倍率无效(42001=", r, ")，暂停等待有效值")
			-- 缓存各状态地址当前状态值
			local armStatus = getRegisterInt("49012")
			local robotStatus = getRegisterInt("50014")
			local cmdAck = getRegisterInt("49128")
			setRegisterInt("50008", 1010)      -- 写入错误码
			setRegisterInt("49012", 3)         -- 机械臂状态设为错误
			setRegisterInt("50014", 3)         -- 机器人整体状态设为错误
			setRegisterInt("49128", -1010)     -- 指令应答返回负错误码
			pauseProject()
			-- 暂停恢复后：恢复各状态地址，循环继续检测42001
			setRegisterInt("50008", 0)   -- 清除错误码
			setRegisterInt("49012", armStatus)      -- 机械臂状态恢复
			setRegisterInt("50014", robotStatus)    -- 机器人整体状态恢复
			setRegisterInt("49128", cmdAck)         -- 指令应答恢复
			badCnt = 0
		end
	else
		badCnt = 0
	end
	-- 仅在倍率有效时执行负载切换，禁止用无效倍率(0)调用setMoveRate
	-- 无效期间的抓取转变不丢失：g_cellGrabbed/bPayload状态保持，有效后下一轮自动补上
	if curRate > 0 then
		if g_cellGrabbed == true and bPayload == false then
			setPayload("2")
			setMoveRate(curRate)                      -- 恢复动态倍率
			bPayload = true
			print("--设置负载为：2")
		elseif g_cellGrabbed == false and bPayload == true then
			setPayload("1")
			setMoveRate(math.min(90, curRate + 10))   -- 空手+10，封顶90
			bPayload = false
			print("--设置负载为：1")
		end
	end
end
