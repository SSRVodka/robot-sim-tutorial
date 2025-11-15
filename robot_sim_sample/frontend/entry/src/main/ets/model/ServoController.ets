export class NativeServoController {
  private nativeInstance: any = null

  constructor() {
    // Initialize native module
    try {
      this.nativeInstance = globalThis.requireNapi('servo_controller')
    } catch (error) {
      console.error('Failed to load native servo controller module:', error)
      throw error
    }
  }

  initialize(): void {
    if (this.nativeInstance) {
      return this.nativeInstance.initialize()
    }
    throw new Error('Native instance not available')
  }

  shutdown(): void {
    if (this.nativeInstance) {
      this.nativeInstance.shutdown()
    }
  }

  sendCartesianCommand(x: number, y: number, z: number, rx: number, ry: number, rz: number): void {
    if (this.nativeInstance) {
      return this.nativeInstance.sendCartesianCommand(x, y, z, rx, ry, rz)
    }
    throw new Error('Native instance not available')
  }

  sendJointCommand(jointIndex: number): void {
    if (this.nativeInstance) {
      return this.nativeInstance.sendJointCommand(jointIndex)
    }
    throw new Error('Native instance not available')
  }

  switchFrame(useEndEffector: boolean): void {
    if (this.nativeInstance) {
      return this.nativeInstance.switchFrame(useEndEffector)
    }
    throw new Error('Native instance not available')
  }

  toggleJointDirection(): void {
    if (this.nativeInstance) {
      return this.nativeInstance.toggleJointDirection()
    }
    throw new Error('Native instance not available')
  }

  getStatus(): { connected: boolean, currentFrame: string, jointDirection: number } {
    if (this.nativeInstance) {
      return this.nativeInstance.getStatus()
    }
    return { connected: false, currentFrame: 'Unknown', jointDirection: 1 }
  }
}