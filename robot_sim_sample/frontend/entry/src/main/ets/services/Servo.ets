
import logger from '../model/Logger';
import libServoGrpc from 'libentry.so';

export class ServoController {
  private isConnected: boolean = false
  private serverAddress: string = 'localhost:50051'
  private currentFrame: 'base' | 'endEffector' = 'base'
  private movementScale: number = 1.0
  
  constructor() {
    // Initialize native module
    logger.info('ServoController initialized')
  }
  
  async connect(address?: string): Promise<boolean> {
    if (address) {
      this.serverAddress = address
    }
    
    try {
      this.isConnected = libServoGrpc.connect(this.serverAddress)
      logger.info(`Connection ${this.isConnected ? 'successful' : 'failed'} to ${this.serverAddress}`)
      return this.isConnected
    } catch (error) {
      logger.error('Failed to connect to servo server:', error)
      this.isConnected = false
      return false
    }
  }
  
  sendTwistCommand(linearX: number, linearY: number, linearZ: number = 0, 
                  angularX: number = 0, angularY: number = 0, angularZ: number = 0): boolean {
    if (!this.isConnected) {
      logger.warn('Not connected to servo server')
      return false
    }
    
    try {
      logger.info(`Sending twist command: ${linearX}, ${linearY}, ${linearZ}, ${angularX}, ${angularY}, ${angularZ} with scale ${this.movementScale}`)
      return libServoGrpc.sendTwist(
        linearX * this.movementScale,
        linearY * this.movementScale,
        linearZ * this.movementScale,
        angularX * this.movementScale,
        angularY * this.movementScale,
        angularZ * this.movementScale
      )
    } catch (error) {
      logger.error('Failed to send twist command:', error)
      return false
    }
  }
  
  sendJointCommand(jointName: string, velocity: number = 1.0): boolean {
    if (!this.isConnected) {
      logger.warn('Not connected to servo server')
      return false
    }
    
    try {
      logger.info(`Sending joint command: ${jointName} with velocity ${velocity} and scale ${this.movementScale}`)
      return libServoGrpc.sendJoint(jointName, velocity * this.movementScale)
    } catch (error) {
      logger.error('Failed to send joint command:', error)
      return false
    }
  }
  
  setReferenceFrame(frame: 'base' | 'endEffector'): boolean {
    if (!this.isConnected) {
      logger.warn('Not connected to servo server')
      return false
    }
    
    try {
      const useEndEffector = frame === 'endEffector'
      const success: boolean = libServoGrpc.setFrame(useEndEffector)
      if (success) {
        this.currentFrame = frame
      }
      logger.info(`Reference frame set to ${frame}: ${success ? 'successful' : 'failed'}`)
      return success
    } catch (error) {
      logger.error('Failed to set reference frame:', error)
      return false
    }
  }
  
  reverseJointDirection(): boolean {
    if (!this.isConnected) {
      logger.warn('Not connected to servo server')
      return false
    }
    
    try {
      logger.info('Reversing joint direction')
      return libServoGrpc.reverseDirection()
    } catch (error) {
      logger.error('Failed to reverse joint direction:', error)
      return false
    }
  }
  
  // Joystick movement handler
  handleJoystickMove(x: number, y: number, isRotational: boolean = false): void {
    logger.info(`Joystick move: ${x}, ${y}`)
    if (!isRotational) {
      // Translational movement (X=forward/back, Y=left/right)
      this.sendTwistCommand(x, y, 0)
    } else {
      // Rotational movement
      this.sendTwistCommand(0, 0, 0, 0, 0, x) // Use x for rotation around Z-axis
    }
  }
  
  // Stop all movement
  stopMovement(): void {
    logger.info('Stopping all movement')
    this.sendTwistCommand(0, 0, 0, 0, 0, 0)
  }
  
  // Getters
  getConnectionStatus(): boolean {
    return this.isConnected
  }
  
  getCurrentFrame(): string {
    return this.currentFrame
  }
  
  getServerAddress(): string {
    return this.serverAddress
  }
  
  setMovementScale(scale: number): void {
    this.movementScale = Math.max(0.1, Math.min(2.0, scale)) // Clamp between 0.1 and 2.0
  }
  
  getMovementScale(): number {
    return this.movementScale
  }
}