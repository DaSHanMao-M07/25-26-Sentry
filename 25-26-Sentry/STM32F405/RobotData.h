#pragma once

class RobotEvent
{
	//详细描述请翻阅《RoboMaster 2021 裁判系统串口协议附录 V1.6（20231122）》
public:
	struct {
		uint8_t Sof;	//1 起始字节，固定0xA5
		uint16_t data_length;  //2 数据长度
		uint8_t seq;	//3 包序号
		uint8_t CRC8;	//4 帧头校验
	}Rxheader;

	//0x0001 比赛状态数据， 频率：1hz  全体机器人 常规链路 
	struct {
		uint8_t game_type : 4;       //1 比赛类型
		uint8_t game_progress : 4;   //2 当前比赛阶段
		uint16_t stage_remain_time;  //3 剩余时间
		uint64_t SyncTimeStamp;		 //4 UNIX时间
	}game_status_t;

	//0x0002 比赛结果数据  频率：结束触发  全体机器人 常规链路
	struct {
		uint8_t winner;  // 0平局 1红胜 2蓝胜
	}game_result_t;

	//0x0003 机器人血量数据  频率：3hz   全体机器人  常规链路
	//若该机器人未上场或者被罚下，则血量为 0
	struct {
		uint16_t red_1_robot_HP;  //1 红 1 英雄机器人血量。
		uint16_t red_2_robot_HP;  //2 红 2 工程机器人血量
		uint16_t red_3_robot_HP;  //3 红 3 步兵机器人血量
		uint16_t red_4_robot_HP;  //4 红 4 步兵机器人血量
		uint16_t red_5_robot_HP;  //5 红 5 步兵机器人血量
		uint16_t red_7_robot_HP;  //6 红 7 哨兵机器人血量
		uint16_t red_outpost_HP;  //7 红方前哨站血量
		uint16_t red_base_HP;     //8 红方基地血量
		uint16_t blue_1_robot_HP; //9 蓝 1 英雄机器人血量
		uint16_t blue_2_robot_HP; //10 蓝 2 工程机器人血量
		uint16_t blue_3_robot_HP; //11 蓝 3 步兵机器人血量
		uint16_t blue_4_robot_HP; //12 蓝 4 步兵机器人血量
		uint16_t blue_5_robot_HP; //13 蓝 5 步兵机器人血量
		uint16_t blue_7_robot_HP; //14 蓝 7 哨兵机器人血量
		uint16_t blue_outpost_HP; //15 蓝方前哨站血量
		uint16_t blue_base_HP;    //16 蓝方基地血量
	}game_robot_HP_t;

	//0x0101  己方场地事件数据 频率：1hz  己方全体机器人  常规链路
	struct {
		uint32_t event_data; //补给站，能量机关，高地，虚拟护盾，敌方飞镖击中时间
	}event_data_t;

	//0x0102  补给站动作标识数据  频率：补给站弹丸释放时触发发送  己方全体机器人  常规链路
	struct {
		uint8_t reserved;				 //1 保留
		uint8_t supply_robot_id;		 //2 补弹机器人ID
		uint8_t supply_projectile_step;  //3 出弹口开闭状态
		uint8_t supply_projectile_num;   //4 补弹数量
	}ext_supply_projectile_action_t;

	//0x0104  裁判警告数据  频率：己方判罚/判负时发送，其余1hz  被处罚方全体机器人  常规链路
	struct {
		uint8_t level;					//1 判罚等级
		uint8_t offending_robot_id;		//2 判罚机器人ID
		uint8_t count;					//3 判罚机器人违规次数
	}referee_warning_t;

	//0x0105  飞镖发射相关数据  频率：1hz  己方全体机器人  常规链路
	struct {
		uint8_t dart_remaining_time;	//1 己方飞镖发射剩余时间，单位：秒
		uint8_t dart_aim_state;			//2 击打目标相关数据
	}dart_info_t;

	//0x0201 机器人性能体系数据  频率：10hz  对应机器人 常规链路
	struct {
		uint8_t robot_id;							  //1 本机器人ID
		uint16_t current_exp;						  //2 机器人当前经验值
		uint16_t current_HP;						  //3 机器人当前血量
		uint8_t power_management_gimbal_output : 1;	  //4 gimbal口电源输出
		uint8_t power_management_chassis_output : 1;  //5 chassis口电源输出
		uint8_t power_management_shooter_output : 1;  //6 shooter口电源输出
	}robot_status_t;

	//0x0202  实时功率热量数据  频率：50hz  对应机器人 常规链路
	struct {
		uint16_t chassis_voltage;				//1 电源管理模块 chassis 口输出电压（单位：mV）
		uint16_t chassis_current;				//2 电源管理模块 chassis 口输出电流（单位：mA）
		float chassis_power;					//3 底盘功率（单位：W）
		uint16_t buffer_energy;					//4 缓冲能量（单位：J）
		uint16_t shooter_17mm_1_barrel_heat;	//5 第 1 个 17mm 发射机构的枪口热量
		uint16_t shooter_17mm_2_barrel_heat;	//6 第 2 个发射机构的枪口热量
		uint16_t shooter_42mm_barrel_heat;		//7 42mm 发射机构的枪口热量
	}power_heat_data_t;

	//0x0203 机器人位置数据  频率：哨兵10hz，其余1hz  对应机器人  常规链路
	struct {
		float x;		//1 本机器人位置 x 坐标，单位：m
		float y;		//2 本机器人位置 y 坐标，单位：m
		float angle;	//3 本机器人测速模块朝向，单位：度。正北为 0 度
	}robot_pos_t;

	//0x0204  机器人增益数据  频率：3hz  对应机器人  常规链路
	struct {
		uint8_t recovery_buff;	//1 机器人回血增益（百分比，值为 10 意为每秒回复 10%最大血量）
		uint8_t cooling_buff;	//2 机器人枪口冷却倍率（直接值，值为 5 意味着 5 倍冷却）
		uint8_t defence_buff;	//3 机器人防御增益（百分比，值为 50 意为 50%防御增益）
		uint16_t attack_buff;	//4 机器人攻击增益（百分比，值为 50 意为 50%攻击增益）
	}buff_t;

	//0x0205  空中支援时间数据 频率：1hz 己方空中机器人 常规链路
	struct {
		uint8_t airforce_status;	//1 空中机器人状态（0 为正在冷却，1 为冷却完毕，2 为空中支援期间）
		uint8_t time_remain;		//2 此状态的剩余时间（单位为 s，向下取整)
	}air_support_data_t;

	//0x0206  伤害状态数据    频率：伤害发生后发送  对应机器人  常规链路
	struct {
		uint8_t armor_id : 4;				//1 装甲模块或测速模块ID编号，或者数值为0，详情看手册
		uint8_t HP_deduction_reason : 4;	//2 血量变化类型
	}hurt_data_t;

	//0x0207 实时射击数据    频率：弹丸发射后发送  对应机器人  常规链路
	struct {
		uint8_t bullet_type;			//1 弹丸类型
		uint8_t shooter_number;			//2 发射机构 ID
		uint8_t launching_frequency;	//3 弹丸射速（单位：Hz）
		float initial_speed;			//4 弹丸初速度（单位：m/s）
	}shoot_data_t;

	//0x0208  允许发弹量  频率：10hz  己方除工程外所有机器人   常规链路
	struct {
		uint16_t projectile_allowance_17mm;		//1 17mm 弹丸允许发弹量
		uint16_t projectile_allowance_42mm;		//2 42mm 弹丸允许发弹量
		uint16_t remaining_gold_coin;			//3 剩余金币数量
	}projectile_allowance_t;

	//0x0209  机器人 RFID 状态  频率：3hz  己方机器人  常规链路
	struct {
		uint32_t rfid_status;	//场地不同地方是否已检测到该增益点 RFID
	}rfid_status_t;

	//0x020A  飞镖选手端指令数据  频率：闸门上线后3hz  己方飞镖  常规链路
	struct {
		uint8_t dart_launch_opening_status;		//1 当前飞镖发射站的状态
		uint8_t dart_attack_target;				//2 飞镖的打击目标：（默认为前哨站）
		uint16_t target_change_time;			//3 切换打击目标时的比赛剩余时间，单位：s，无未切换动作默认为 0
		uint16_t latest_launch_cmd_time;		//4 最后一次操作手确定发射指令时的比赛剩余时间，单位：s，初始值为 0
	}dart_client_cmd_t;

	//0x020B  地面机器人位置数据  频率：1hz  己方哨兵机器人  常规链路
	struct {
		float hero_x;			//1 己方英雄机器人位置 x 轴坐标，单位：m
		float hero_y;			//2 己方英雄机器人位置 y 轴坐标，单位：m
		float engineer_x;		//3 己方工程机器人位置 x 轴坐标，单位：m
		float engineer_y;		//4 己方工程机器人位置 y 轴坐标，单位：m
		float standard_3_x;		//5 己方 3 号步兵机器人位置 x 轴坐标，单位：m
		float standard_3_y;		//6 己方 3 号步兵机器人位置 y 轴坐标，单位：m
		float standard_4_x;		//7 己方 4 号步兵机器人位置 x 轴坐标，单位：m
		float standard_4_y;		//8 己方 4 号步兵机器人位置 y 轴坐标，单位：m
		float standard_5_x;		//9 己方 5 号步兵机器人位置 x 轴坐标，单位：m
		float standard_5_y;		//10 己方 5 号步兵机器人位置 y 轴坐标，单位：m
	}ground_robot_position_t;

	//0x020C 雷达标记进度数据 频率：1hz  己方雷达机器人  常规链路
	struct {
		uint8_t mark_hero_progress;			//1 对方英雄机器人被标记进度：0—120
		uint8_t mark_engineer_progress;		//2 对方工程机器人被标记进度：0—120
		uint8_t mark_standard_3_progress;	//3 对方 3 号步兵机器人被标记进度：0—120
		uint8_t mark_standard_4_progress;	//4 对方 4 号步兵机器人被标记进度：0—120
		uint8_t mark_standard_5_progress;	//5 对方 5 号步兵机器人被标记进度：0—120
		uint8_t mark_sentry_progress;		//6 对方哨兵机器人被标记进度：0—120
	}radar_mark_data_t;

	//0x020D 哨兵自主决策相关信息同步 频率：1hz  己方哨兵机器人 常规链路
	struct {
		uint32_t sentry_info;  //兑换发单量，兑换血量次数
	}sentry_info_t;

	//0x020E 雷达自主决策信息同步  频率：1hz  己方雷达机器人  常规链路
	struct {
		uint8_t radar_info;
	}radar_info_t;

	//0x0301  机器人交互数据  频率：发送方触发发送，上限10hz  常规链路
	struct {
		uint16_t data_cmd_id;	//1 子内容 ID,需为开放的子内容 ID
		uint16_t sender_id;		//2 发送者 ID,需与自身 ID 匹配，ID 编号详见附录
		uint16_t receiver_id;	//3 接收者 ID,ID 编号详见附录
		uint8_t* user_data;	    //4 x 最大为 113
	}robot_interaction_data_t;

	//0x0303  选手端小地图交互数据  频率：云台手触发发送  常规链路
	struct {
		float target_position_x;	//1 目标位置 x 轴坐标，单位 m,当发送目标机器人 ID 时，该值为 0
		float target_position_y;	//2 目标位置 y 轴坐标，单位 m,当发送目标机器人 ID 时，该值为 0
		uint8_t cmd_keyboard;		//3 云台手按下的键盘按键通用键值,无按键为0
		uint8_t target_robot_id;	//4 对方机器人 ID
		uint16_t cmd_source;			//5 信息来源 ID，见附录
	}map_command_t;
};

//0x0305  选手端小地图接收雷达数据  频率：上限10hz  常规链路
struct map_robot_data_t {
	uint16_t target_robot_id;	//1 目标机器人 ID
	float target_position_x;	//2 目标 x 位置坐标，单位：m，超出不显示
	float target_position_y;	//3 目标 x 位置坐标，单位：m，超出不显示
};

//0x0307 选手端小地图接收哨兵数据  频率：1hz 常规链路
struct map_data_t {
	uint8_t intention;			//1 移动到目标点，可以攻击或者防守
	uint16_t start_position_x;	//2 路径起点 x 轴坐标，单位：dm
	uint16_t start_position_y;	//3 路径起点 y 轴坐标，单位：dm
	int8_t delta_x[49];			//4 路径点 x 轴增量数组，单位：dm
	int8_t delta_y[49];			//5 路径点 y 轴增量数组，单位：dm
	uint16_t sender_id;			//6 发送者 ID，需与自身 ID 匹配，ID 编号详见附录
};

//0x0308 选手端小地图接收机器人消息 3hz   常规链路
struct custom_info_t {
	uint16_t sender_id;
	uint16_t receiver_id;
	uint16_t user_data[30];
};

//图传链路数据，自定义控制器

//0x0302 自定义控制器与机器人交互数据  <30Hz
struct custom_robot_data_t {
	uint8_t data[20];
};

//0x0304 键鼠遥控数据 30hz
//将通过遥控器发送的键鼠遥控数据同步通过图传链路发送给对应机器人。
struct remote_control_t {
	int16_t mouse_x;			//鼠标 x 轴移动速度，负值标识向左移动
	int16_t mouse_y;			//鼠标 y 轴移动速度，负值标识向下移动
	int16_t mouse_z;			//鼠标滚轮移动速度，负值标识向后滚动
	int8_t left_button_down;	//鼠标左键是否按下：0 为没按下；1 为按下
	int8_t right_button_down;	//鼠标右键是否按下：0 为没按下，1 为按下
	uint16_t keyboard_value;
	uint16_t reserved;
};


//0x0301的交互数据内容
//		|
//		|
//		v


//0x0100 选手端删除图层
struct interaction_layer_delete_t {
	uint8_t delete_type;	//删除操作
	uint8_t layer;			//图层数
};

//0x0101 选手端绘制一个图形
//0x0102  选手端绘制二个图形
//0x0103 选手端绘制五个图形
//0x0104  选手端绘制七个图形
struct interaction_figure_t {
	uint8_t figure_name[3];		//图形名,在图形删除，修改等操作中，作为索引
	uint32_t operate_tpye : 3;	//图形操作
	uint32_t figure_tpye : 3;	//图形类型
	uint32_t layer : 4;			//图层数 0-9
	uint32_t color : 4;			//颜色
	uint32_t details_a : 9;		//依绘制的不同图形含义不同
	uint32_t details_b : 9;		//依绘制的不同图形含义不同
	uint32_t width : 10;		//线宽，建议字体大小与线宽比例为 10：1
	uint32_t start_x : 11;		//起点/圆心 x 坐标
	uint32_t start_y : 11;		//起点/圆心 y 坐标
	uint32_t details_c : 10;	//依绘制的不同图形含义不同
	uint32_t details_d : 11;	//依绘制的不同图形含义不同
	uint32_t details_e : 11;	//依绘制的不同图形含义不同
};



//0x0110 选手端绘制字符图形
struct ext_client_custom_character_t {
	interaction_figure_t grapic_data_struct;
	uint8_t data[30];
};

//0x0120 哨兵自主决策指令
struct sentry_cmd_t {
	uint32_t sentry_cmd;
};

//0x0121 雷达自主决策指令
struct radar_cmd_t {
	uint8_t radar_cmd;
};