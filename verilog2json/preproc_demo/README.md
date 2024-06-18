# Feature 

1. include 嵌套
   arbiter.v line 26 
   config.vh line 1 
2. 不同的.vh，存在同名 define
    arbiter_config_0.vh 和 arbiter_config_1.vh
3. define 嵌套 define
    config.vh line 3,4
4. inlcude .v文件
    arbiter.v line 27
4. 多次define覆盖
    config.vh line 4
    priority_encoder.v line 27
5. define 和 undef 作用域
    config.vh line 8
    priority_encoder.v line 28
6. parameter 嵌套 define
    arbiter.v line 43
7. parameter 嵌套 parameter
    arbiter.v line 58
8. port width使用$clog2
    arbiter.v line 56

# Use
    make sim
    make verdi
    make clean


