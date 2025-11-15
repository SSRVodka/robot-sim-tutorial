import router from '@ohos.router';

import { JoystickComponent } from '../components/Joystick';
import { ServoController } from '../services/Servo';

import logger from '../model/Logger';

@Entry
@Component
struct JoystickPage {
  @State private controller: ServoController = new ServoController()
  @State private isConnected: boolean = false
  @State private serverAddress: string = '192.168.144.1:50052'
  @State private currentFrame: string = 'base'
  @State private movementScale: number = 1.0
  @State private selectedJoint: string = 'panda_joint1'
  
  private joints: string[] = [
    'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
    'panda_joint5', 'panda_joint6', 'panda_joint7'
  ]
  
  aboutToAppear() {
    logger.info('JoystickPage aboutToAppear')
  }
  
  private async connectToServer() {
    this.isConnected = await this.controller.connect(this.serverAddress)
    if (this.isConnected) {
      this.currentFrame = this.controller.getCurrentFrame()
    }
  }
  
  private onMainJoystickMove = (x: number, y: number) => {
    this.controller.handleJoystickMove(x, y, false)
  }
  
  private onMainJoystickRelease = () => {
    this.controller.stopMovement()
  }
  
  private onRotationJoystickMove = (x: number, y: number) => {
    this.controller.handleJoystickMove(x, y, true)
  }
  
  private onRotationJoystickRelease = () => {
    this.controller.stopMovement()
  }

  private navigateToChat() {
    router.pushUrl({
      url: 'pages/ChatPage'
    })
  }
  
  build() {
    Column() {
      // Header with navigation
      Row() {
        Text('Servo Joystick Controller')
          .fontSize(24)
          .fontWeight(FontWeight.Bold)
          .flexGrow(1)
        
        Button('Chat →')
          .backgroundColor('#FF9800')
          .fontSize(16)
          .margin({ right: 10 })
          .onClick(() => {
            this.navigateToChat()
          })
        
        Circle()
          .width(12)
          .height(12)
          .fill(this.isConnected ? '#4CAF50' : '#F44336')
      }
      .width('100%')
      .justifyContent(FlexAlign.SpaceBetween)
      .padding({ left: 20, right: 20, top: 20 })
      
      // Connection Section
      Column() {
        Row() {
          TextInput({ text: this.serverAddress })
            .width('70%')
            .onChange((value) => {
              this.serverAddress = value
            })
          
          Button(this.isConnected ? 'Disconnect' : 'Connect')
            .width('25%')
            .type(ButtonType.Normal)
            .backgroundColor(this.isConnected ? '#F44336' : '#2196F3')
            .onClick(() => {
              if (this.isConnected) {
                this.isConnected = false
              } else {
                this.connectToServer()
              }
            })
        }
        .width('100%')
        .justifyContent(FlexAlign.SpaceBetween)
      }
      .padding(20)
      
      // Control Panel
      if (this.isConnected) {
        Column() {
          // Frame Selection
          Row() {
            Text('Reference Frame:')
              .fontSize(16)
              .margin({ right: 10 })
            
            Button('Base Frame')
              .backgroundColor(this.currentFrame === 'base' ? '#1976D2' : '#E0E0E0')
              .fontColor(this.currentFrame === 'base' ? '#FFFFFF' : '#000000')
              .onClick(() => {
                if (this.controller.setReferenceFrame('base')) {
                  this.currentFrame = 'base'
                }
              })
            
            Button('End-Effector')
              .backgroundColor(this.currentFrame === 'endEffector' ? '#1976D2' : '#E0E0E0')
              .fontColor(this.currentFrame === 'endEffector' ? '#FFFFFF' : '#000000')
              .margin({ left: 10 })
              .onClick(() => {
                if (this.controller.setReferenceFrame('endEffector')) {
                  this.currentFrame = 'endEffector'
                }
              })
          }
          .width('100%')
          .justifyContent(FlexAlign.Start)
          .margin({ bottom: 20 })
          
          // Movement Scale
          Row() {
            Text('Speed:')
              .fontSize(16)
              .margin({ right: 10 })
            
            Slider({
              value: this.movementScale * 100,
              min: 10,
              max: 200,
              style: SliderStyle.OutSet
            })
              .width('60%')
              .onChange((value) => {
                this.movementScale = value / 100
                this.controller.setMovementScale(this.movementScale)
              })
            
            Text(`${(this.movementScale * 100).toFixed(0)}%`)
              .fontSize(16)
              .margin({ left: 10 })
          }
          .width('100%')
          .justifyContent(FlexAlign.Start)
          .margin({ bottom: 20 })
          
          // Main Joysticks
          Row() {
            // Translation Joystick
            Column() {
              Text('Translation')
                .fontSize(16)
                .fontWeight(FontWeight.Medium)
                .margin({ bottom: 10 })
              
              JoystickComponent({
                onJoystickMove: this.onMainJoystickMove,
                onJoystickRelease: this.onMainJoystickRelease
              })
            }
            .width('45%')
            
            Blank().width('10%')
            
            // Rotation Joystick
            Column() {
              Text('Rotation')
                .fontSize(16)
                .fontWeight(FontWeight.Medium)
                .margin({ bottom: 10 })
              
              JoystickComponent({
                onJoystickMove: this.onRotationJoystickMove,
                onJoystickRelease: this.onRotationJoystickRelease
              })
            }
            .width('45%')
          }
          .width('100%')
          .margin({ bottom: 30 })
          
          // Z-axis controls
          Row() {
            Button('Up')
              .width('20%')
              .backgroundColor('#4CAF50')
              .gesture(
                TapGesture()
                  .onAction(() => {
                    this.controller.sendTwistCommand(0, 0, 1)
                    setTimeout(() => this.controller.stopMovement(), 200)
                  })
              )
            
            Blank().width('10%')
            
            Button('Down')
              .width('20%')
              .backgroundColor('#FF9800')
              .gesture(
                TapGesture()
                  .onAction(() => {
                    this.controller.sendTwistCommand(0, 0, -1)
                    setTimeout(() => this.controller.stopMovement(), 200)
                  })
              )
            
            Blank().width('10%')
            
            Button('Stop All')
              .width('30%')
              .backgroundColor('#F44336')
              .onClick(() => {
                this.controller.stopMovement()
              })
          }
          .width('100%')
          .margin({ bottom: 20 })
          
          // Joint Control Section
          Column() {
            Text('Joint Control')
              .fontSize(18)
              .fontWeight(FontWeight.Bold)
              .margin({ bottom: 15 })
            
            // Joint Selection
            Row() {
              Text('Joint:')
                .fontSize(16)
                .margin({ right: 10 })
              
              Select(this.joints.map(joint => ({ value: joint } as SelectOption)))
                .selected(this.joints.indexOf(this.selectedJoint))
                .value(this.selectedJoint)
                .onSelect((index) => {
                  this.selectedJoint = this.joints[index]
                })
            }
            .width('100%')
            .justifyContent(FlexAlign.Start)
            .margin({ bottom: 15 })
            
            // Joint movement buttons
            Row() {
              Button('←')
                .width('20%')
                .gesture(
                  TapGesture()
                    .onAction(() => {
                      this.controller.sendJointCommand(this.selectedJoint, -1)
                      setTimeout(() => this.controller.stopMovement(), 200)
                    })
                )
              
              Blank().width('10%')
              
              Button('→')
                .width('20%')
                .gesture(
                  TapGesture()
                    .onAction(() => {
                      this.controller.sendJointCommand(this.selectedJoint, 1)
                      setTimeout(() => this.controller.stopMovement(), 200)
                    })
                )
              
              Blank().width('10%')
              
              Button('Reverse Dir')
                .width('30%')
                .backgroundColor('#9C27B0')
                .onClick(() => {
                  this.controller.reverseJointDirection()
                })
            }
            .width('100%')
          }
          .padding(15)
          .backgroundColor('#F5F5F5')
          .borderRadius(8)
        }
        .padding(20)
      }
      
      Blank()
    }
    .width('100%')
    .height('100%')
    .backgroundColor('#FAFAFA')
  }
}