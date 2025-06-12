## Rtsi
- 描述：RTSI客户端

### bool connect(const std::string& ip, int port, ErrorCode* ec = nullptr)
- 描述：连接控制RTSI
- 输入参数：
    - ip：机器人IP
    - port：RTSI端口（即30004）
    - ec：错误信息
- 返回值：成功为true

###  bool connect(const std::string& ip);
- 描述：连接控制RTSI
- 输入参数：
    - ip：机器人IP
    - port：RTSI端口（即30004）
    - ec：错误信息
- 返回值：成功为true

### void disconnect()
- 描述：断开与机器人的连接

### bool versionCheck()
- 描述：RTSI 协议检查（版本1）
- 返回值：成功为true


### ControllerVsersion controllerVersionGet(bool need_conver_to_little_endian = true)
- 描述：获取控制器版本
- 输入参数：
    - need_conver_to_little_endian：是否将返回结果转换为小端
- 返回值：控制器版本
- 注：返回值结构体声明如下
    ```
    struct ControllerVsersion {
        uint32_t major;
        uint32_t minor;
        uint32_t bugfix;
        uint32_t build;
    };
    ```

### bool start()
- 描述：发送开始信号
- 返回值：成功为true

### bool pause()
- 描述：发送开始信号
- 返回值：成功为true


### ConnectionState state()
- 描述：客户端状态
- 返回值：客户端状态
- 注：返回值声明如下
    ```
    enum ConnectionState {
        DISCONNECTED,
        CONNECTED,
        STARTED,
        STOPED
    };
    ```

### DataRecipePtr outputSubscribe(const std::string& items, double frequency, ErrorCode* ec = nullptr)
- 描述：设置RTSI输出订阅
- 输入参数：
    - items：订阅项字符串，字符串需要符合RTSI订阅项标准
    - frequency：输出频率
    - ec：错误信息
- 返回值：订阅的配方
- 注：DataRecipePtr 的声明为 `typedef std::shared_ptr<DataRecipe> DataRecipePtr;`

### DataRecipePtr inputSubscribe(const std::string& items, ErrorCode* ec = nullptr)
- 描述：设置RTSI输入订阅
- 输入参数：
    - items：订阅项字符串，字符串需要符合RTSI订阅项标准
    - ec：错误信息
- 返回值：订阅的配方
- 注：DataRecipePtr 的声明为 `typedef std::shared_ptr<DataRecipe> DataRecipePtr;`


### bool getOutputData(const DataRecipePtr& recipe_ptr, ErrorCode* ec = nullptr)
- 描述：获取输出订阅的数据
- 输入参数：
    - recipe_ptr：订阅的配方
    - ec：错误信息
- 返回值：成功为true
- 注：此接口适用于输出订阅单配方的情况


### DataRecipePtr& getOutputDataToRecipe(ErrorCode* ec = nullptr)
- 描述：获取输出订阅的数据
- 输入参数：
    - ec：错误信息
- 返回值：订阅的配方
- 注：此接口适用于单配方和多配方的情况

### bool getOutPutDataToRaw(RtsiMsg::DataTxRx* data_buff, int max_buff_size, ErrorCode* ec = nullptr)
- 描述：获取输出订阅的原始数据
- 输入参数：
    - data_buff：获取到的原始报文缓存
    - max_buff_size：缓存的最大容量（字节）
    - ec：错误信息
- 返回值：成功为true

### bool sendInputData(const DataRecipePtr& recipe_ptr, ErrorCode* ec = nullptr)
- 描述：发送输入订阅项的数据
- 输入参数：
    - recipe_ptr：输入订阅项配方
    - ec：错误信息
- 返回值：成功为true

### bool sendInputDataFromRaw(const RtsiMsg::DataTxRx& data_buff, int max_buff_size, ErrorCode* ec = nullptr)
- 描述：从原始报文直接发送
- 输入参数：
    - data_buff：原始报文
    - max_buff_size：报文的最大容量
    - ec：错误信息
- 返回值：成功为true
- 注：报文为大端序


### bool sendTextMessage(const std::string& message, const std::string& source, MessageType type)
- 描述：发送文本消息
- 输入参数：
    - message：消息内容
    - source：消息源
    - type:消息类型
- 返回值：成功为true
- 注：消息类型有以下几种
    ```
    enum MessageType{
        EXCEPTION_MESSAGE = 0,
        ERROR_MESSAGE = 1,
        WARNING_MESSAGE = 2,
        INFO_MESSAGE = 3
    };
    ```
    
## Rtsi::DataRecipe
- 描述：RTSI订阅的配方
- 数据结构：
    ```
    struct DataObject {
        union Data {
            double v6d[6];
            double v3d[3];
            int32_t v6i32[6];
            double d_64;
            uint16_t u_16;
            uint32_t u_32;
            uint64_t u_64;
            bool b_8;
            uint8_t u_8;
            int32_t i_32;
        };
        Data value;
        std::string name;
        std::string type;
    };
    typedef std::string DataKey; 
    typedef std::unordered_map<DataKey, DataObject> DataMap;
    typedef std::list<DataKey> ItemList;
    typedef std::list<DataKey>::iterator ItemIterator;
    ```

### DataObject& operator[](const DataKey& name);
- 描述：重载了[]运算符，因此可以像C++ 的 map 类一样直接 recipe["item"] 来获取数据
- 输入参数：
    - name：项目名称
- 返回值：RTSI数据对象


### bool is_empty()
- 描述：是否为空，即没有任何数据项
- 返回值：true为空


### const ItemList& getItemList()
- 描述：获取项目列表
- 返回值：项目列表

### uint8_t getID()
- 描述：获取配方ID
- 返回值：配方ID
- 注：配方ID为0或255则可能有问题

### bool build(uint8_t recipe_id, DataMap dmap, ItemList ilist)
- 描述：构建配方
- 输入参数：
    - recipe_id：配方ID
    - dmap：配方的哈希表
    - ilist：配方的列表
- 返回值：true为成功


bool convertToMessage(RtsiMsg::DataTxRx* msg_buff, int max_buff_size)
- 描述：将配方里的数据输出为报文
- 输入参数：
    - msg_buff：报文缓存
    - max_buff_size：报文最大缓存字节数
- 返回值：true为成功


### bool setValueFromMessage(const RtsiMsg::DataTxRx& msg_buff)
- 描述：将报文的数据转换为配方
- 输入参数：
    - msg_buff：报文
- 返回值：true为成功

## bool has_not_found()
- 描述：是否存在NOT_FOUND项目
- 返回值：true为存在

### bool has_in_use()
- 描述：是否存在IN_USE项目
- 返回值：true为存在
