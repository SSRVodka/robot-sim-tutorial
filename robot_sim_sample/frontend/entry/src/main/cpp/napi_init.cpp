#include <napi/native_api.h>
#include <grpcpp/grpcpp.h>
#include <memory>
#include <string>
#include <mutex>

#include "agent_control.grpc.pb.h"
#include "servo_control.grpc.pb.h"
#include "utils.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

class ServoGrpcClient {
private:
    std::unique_ptr<servo_control::ServoControl::Stub> stub_;
    std::mutex mutex_;
    bool connected_;

public:
    ServoGrpcClient() : connected_(false) {}
    
    bool connect(const std::string& server_address) {
        std::lock_guard<std::mutex> lock(mutex_);
        try {
            auto channel = grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials());
            // actively wait for the connection to be ready
            if (!channel->WaitForConnected(gpr_time_add(
                    gpr_now(GPR_CLOCK_REALTIME),
                    gpr_time_from_seconds(3, GPR_TIMESPAN)))) {
                connected_ = false;
                return false;
            }
            stub_ = servo_control::ServoControl::NewStub(channel);
            connected_ = true;
            return true;
        } catch (...) {
            connected_ = false;
            return false;
        }
    }
    
    bool sendTwistCommand(double linear_x, double linear_y, double linear_z,
                         double angular_x = 0, double angular_y = 0, double angular_z = 0) {
        if (!connected_) return false;
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        servo_control::TwistCommand request;
        request.set_linear_x(linear_x);
        request.set_linear_y(linear_y);
        request.set_linear_z(linear_z);
        request.set_angular_x(angular_x);
        request.set_angular_y(angular_y);
        request.set_angular_z(angular_z);
        
        servo_control::CommandResponse response;
        ClientContext context;
        
        Status status = stub_->SendTwistCommand(&context, request, &response);
        LOGI("ServoControl.SendTwistCommand: %{public}s; Response: %{public}s", status.ok() ? "OK" : "FAILED", response.success() ? "OK" : "FAILED");
        return status.ok() && response.success();
    }
    
    bool sendJointCommand(const std::string& joint_name, double velocity) {
        if (!connected_) return false;
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        servo_control::JointCommand request;
        request.set_joint_name(joint_name);
        request.set_velocity(velocity);
        
        servo_control::CommandResponse response;
        ClientContext context;
        
        Status status = stub_->SendJointCommand(&context, request, &response);
        LOGI("ServoControl.SendJointCommand: %{public}s; Response: %{public}s", status.ok() ? "OK" : "FAILED", response.success() ? "OK" : "FAILED");
        return status.ok() && response.success();
    }
    
    bool setReferenceFrame(bool use_end_effector) {
        if (!connected_) return false;
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        servo_control::FrameCommand request;
        request.set_frame(use_end_effector ? 
            servo_control::FrameCommand::END_EFFECTOR_FRAME : 
            servo_control::FrameCommand::BASE_FRAME);
        
        servo_control::CommandResponse response;
        ClientContext context;
        
        Status status = stub_->SetReferenceFrame(&context, request, &response);
        return status.ok() && response.success();
    }
    
    bool reverseJointDirection() {
        if (!connected_) return false;
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        servo_control::Empty request;
        servo_control::CommandResponse response;
        ClientContext context;
        
        Status status = stub_->ReverseJointDirection(&context, request, &response);
        return status.ok() && response.success();
    }
};


class AgentGrpcClient {
    std::unique_ptr<agent_control::AgentControl::Stub> stub_;
    std::mutex mutex_;
    bool connected_;
public:
    AgentGrpcClient() : connected_(false) {}
    bool connect(const std::string& server_address) {
        std::lock_guard<std::mutex> lock(mutex_);
        try {
            auto channel = grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials());
            // actively wait for the connection to be ready
            if (!channel->WaitForConnected(gpr_time_add(
                    gpr_now(GPR_CLOCK_REALTIME),
                    gpr_time_from_seconds(3, GPR_TIMESPAN)))) {
                connected_ = false;
                return false;
            }
            stub_ = agent_control::AgentControl::NewStub(channel);
            connected_ = true;
            return true;
        } catch (...) {
            connected_ = false;
            return false;
        }
    }

    std::string sendPrompt(const std::string &message) {
        if (!connected_) return "";
        
        std::lock_guard<std::mutex> lock(mutex_);
        agent_control::AgentCommand request;
        agent_control::CommandResponse response;
        request.set_command(message);
        ClientContext context;

        Status status = stub_->SendPrompt(&context, request, &response);
        LOGI("AgentControl.SendPrompt: %{public}s; Response: %{public}s", status.ok() ? "OK" : "FAILED", response.success() ? "OK" : "FAILED");
        return response.message();
    }
};


static ServoGrpcClient* g_client = nullptr;
static AgentGrpcClient* g_agent_client = nullptr;

// NAPI wrapper functions
static napi_value Connect(napi_env env, napi_callback_info info) {
    size_t argc = 1;
    napi_value args[1];
    napi_get_cb_info(env, info, &argc, args, nullptr, nullptr);
    
    size_t str_len;
    napi_get_value_string_utf8(env, args[0], nullptr, 0, &str_len);
    
    char* server_address = new char[str_len + 1];
    napi_get_value_string_utf8(env, args[0], server_address, str_len + 1, &str_len);
    
    if (!g_client) {
        g_client = new ServoGrpcClient();
    }
    
    bool success = g_client->connect(std::string(server_address));
    delete[] server_address;
    
    napi_value result;
    napi_get_boolean(env, success, &result);
    return result;
}

static napi_value SendTwist(napi_env env, napi_callback_info info) {
    size_t argc = 6;
    napi_value args[6];
    napi_get_cb_info(env, info, &argc, args, nullptr, nullptr);
    
    double values[6] = {0};
    for (size_t i = 0; i < argc && i < 6; i++) {
        napi_get_value_double(env, args[i], &values[i]);
    }
    
    bool success = false;
    if (g_client) {
        success = g_client->sendTwistCommand(values[0], values[1], values[2], 
                                           values[3], values[4], values[5]);
    }
    
    napi_value result;
    napi_get_boolean(env, success, &result);
    return result;
}

static napi_value SendJoint(napi_env env, napi_callback_info info) {
    size_t argc = 2;
    napi_value args[2];
    napi_get_cb_info(env, info, &argc, args, nullptr, nullptr);
    
    size_t str_len;
    napi_get_value_string_utf8(env, args[0], nullptr, 0, &str_len);
    
    char* joint_name = new char[str_len + 1];
    napi_get_value_string_utf8(env, args[0], joint_name, str_len + 1, &str_len);
    
    double velocity;
    napi_get_value_double(env, args[1], &velocity);
    
    bool success = false;
    if (g_client) {
        success = g_client->sendJointCommand(std::string(joint_name), velocity);
    }
    
    delete[] joint_name;
    
    napi_value result;
    napi_get_boolean(env, success, &result);
    return result;
}

static napi_value SetFrame(napi_env env, napi_callback_info info) {
    size_t argc = 1;
    napi_value args[1];
    napi_get_cb_info(env, info, &argc, args, nullptr, nullptr);
    
    bool use_end_effector;
    napi_get_value_bool(env, args[0], &use_end_effector);
    
    bool success = false;
    if (g_client) {
        success = g_client->setReferenceFrame(use_end_effector);
    }
    
    napi_value result;
    napi_get_boolean(env, success, &result);
    return result;
}

static napi_value ReverseDirection(napi_env env, napi_callback_info info) {
    bool success = false;
    if (g_client) {
        success = g_client->reverseJointDirection();
    }
    
    napi_value result;
    napi_get_boolean(env, success, &result);
    return result;
}


static napi_value ConnectChat(napi_env env, napi_callback_info info) {
    size_t argc = 1;
    napi_value args[1];
    napi_get_cb_info(env, info, &argc, args, nullptr, nullptr);
    
    size_t str_len;
    napi_get_value_string_utf8(env, args[0], nullptr, 0, &str_len);
    
    char* server_address = new char[str_len + 1];
    napi_get_value_string_utf8(env, args[0], server_address, str_len + 1, &str_len);
    
    if (!g_agent_client) {
        g_agent_client = new AgentGrpcClient();
    }
    
    bool success = g_client->connect(std::string(server_address));
    delete[] server_address;
    
    napi_value result;
    napi_get_boolean(env, success, &result);
    return result;
}
static napi_value SendChat(napi_env env, napi_callback_info info) {
    size_t argc = 1;
    napi_value args[1];
    napi_get_cb_info(env, info, &argc, args, nullptr, nullptr);
    
    size_t str_len;
    napi_get_value_string_utf8(env, args[0], nullptr, 0, &str_len);
    
    char* user_msg = new char[str_len + 1];
    napi_get_value_string_utf8(env, args[0], user_msg, str_len + 1, &str_len);

    std::string response;
    
    if (g_agent_client) {
        response = g_agent_client->sendPrompt(std::string(user_msg));
    }
    
    delete[] user_msg;
    
    napi_value result;
    napi_create_string_utf8(env, response.c_str(), response.size(), &result);
    return result;
}

EXTERN_C_START
static napi_value Init(napi_env env, napi_value exports) {
    napi_property_descriptor desc[] = {
        {"connect", nullptr, Connect, nullptr, nullptr, nullptr, napi_default, nullptr},
        {"sendTwist", nullptr, SendTwist, nullptr, nullptr, nullptr, napi_default, nullptr},
        {"sendJoint", nullptr, SendJoint, nullptr, nullptr, nullptr, napi_default, nullptr},
        {"setFrame", nullptr, SetFrame, nullptr, nullptr, nullptr, napi_default, nullptr},
        {"reverseDirection", nullptr, ReverseDirection, nullptr, nullptr, nullptr, napi_default, nullptr},
        {"connectChat", nullptr, ConnectChat, nullptr, nullptr, nullptr, napi_default, nullptr},
        {"sendChatMessage", nullptr, SendChat, nullptr, nullptr, nullptr, napi_default, nullptr}
    };
    
    napi_define_properties(env, exports, sizeof(desc) / sizeof(desc[0]), desc);
    return exports;
}
EXTERN_C_END

static napi_module demoModule = {
    .nm_version = 1,
    .nm_flags = 0,
    .nm_filename = nullptr,
    .nm_register_func = Init,
    .nm_modname = "entry",
    .nm_priv = ((void*)0),
    .reserved = { 0 },
};

extern "C" __attribute__((constructor)) void RegisterServoGrpcClientModule(void) {
    LOGI("[Init] NAPI Registering...");
    napi_module_register(&demoModule);
}