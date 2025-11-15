// 注：为了方便起见，这里的 model 和 label file 直接使用 sandbox 下的路径，应用会在启动时从 rawfile copy 到 sandbox 中传入（参见 Index.ets）
// 但是 input_path 需要用户指定、用户来控制，因此这里使用 userspace 下的路径，native 程序不能直接访问，需要转换
// （参见 napi_init.cpp::user_uri_to_sandbox_path 的使用）
// 如果开发者希望 model, label file, output_sandbox_path 也作为用户定义的一部分，请自行修改代码。
// 另附：
// - OpenHarmony 沙箱机制：https://gitee.com/openharmony/docs/blob/master/zh-cn/application-dev/file-management/app-sandbox-directory.md
// - ArkTS 应用资源分类：https://gitee.com/openharmony/docs/blob/master/zh-cn/application-dev/quick-start/resource-categories-and-access.md
// - 其他的 tips：https://forums.openharmony.cn/forum.php?mod=viewthread&tid=1222
// export const process_image: (
//     model_sandbox_path: string, label_sandbox_path: string,
//     input_path: string, output_sandbox_path: string) => number;
// export const capture_and_process_image: (
//     model_sandbox_path: string, label_sandbox_path: string,
//     output_sandbox_path: string) => number;

declare module 'libentry.so' {
  /**
   * Connect to the gRPC servo server
   * @param serverAddress Server address in format "host:port"
   * @returns true if connection successful, false otherwise
   */
  function connect(serverAddress: string): boolean;
  
  /**
   * Send twist command for Cartesian movement
   * @param linearX Linear velocity in X direction
   * @param linearY Linear velocity in Y direction  
   * @param linearZ Linear velocity in Z direction
   * @param angularX Angular velocity around X axis
   * @param angularY Angular velocity around Y axis
   * @param angularZ Angular velocity around Z axis
   * @returns true if command sent successfully, false otherwise
   */
  function sendTwist(
    linearX: number,
    linearY: number, 
    linearZ: number,
    angularX: number,
    angularY: number,
    angularZ: number
  ): boolean;
  
  /**
   * Send joint command for individual joint movement
   * @param jointName Name of the joint (e.g., "panda_joint1")
   * @param velocity Joint velocity
   * @returns true if command sent successfully, false otherwise
   */
  function sendJoint(jointName: string, velocity: number): boolean;
  
  /**
   * Set reference frame for twist commands
   * @param useEndEffector true for end-effector frame, false for base frame
   * @returns true if frame set successfully, false otherwise
   */
  function setFrame(useEndEffector: boolean): boolean;
  
  /**
   * Reverse joint movement direction
   * @returns true if direction reversed successfully, false otherwise
   */
  function reverseDirection(): boolean;


  /**
   * Connect to the gRPC AI chat server
   * @param serverAddress Server address in format "host:port"
   * @returns true if connection successful, false otherwise
   */
  function connectChat(serverAddress: string): boolean;
  
  /**
   * Send message to AI chat server and get response
   * @param message User message content
   * @returns AI response string or null if failed
   */
  function sendChatMessage(message: string): Promise<string | null>;
}