import router from '@ohos.router';

import { AiChatController, ChatMessage } from './chat';
import logger from '../model/Logger';

@Entry
@Component
struct AiChatPage {
  @State private controller: AiChatController = new AiChatController()
  @State private isConnected: boolean = false
  @State private serverAddress: string = 'localhost:50051'
  @State private messages: ChatMessage[] = []
  @State private inputText: string = ''
  @State private isLoading: boolean = false
  
  aboutToAppear() {
    logger.info('AiChatPage aboutToAppear')
  }
  
  private async connectToServer() {
    this.isConnected = await this.controller.connect(this.serverAddress)
    if (this.isConnected) {
      this.messages = this.controller.getMessages()
    }
  }
  
  private async sendMessage() {
    if (!this.inputText.trim() || this.isLoading) return
    
    const messageText = this.inputText.trim()
    this.inputText = ''
    this.isLoading = true
    
    try {
      await this.controller.sendMessage(messageText)
      this.messages = this.controller.getMessages()
    } catch (error) {
      logger.error('Failed to send message:', error)
    } finally {
      this.isLoading = false
    }
  }
  
  build() {
    Column() {
      // Header with navigation
      Row() {
        Button('← Joystick')
          .fontSize(16)
          .backgroundColor('#E0E0E0')
          .fontColor('#000000')
          .onClick(() => {
            // Navigate to joystick page
            router.pushUrl({ url: 'pages/JoystickPage' })
          })
        
        Text('AI Chat Assistant')
          .fontSize(24)
          .fontWeight(FontWeight.Bold)
          .layoutWeight(1)
          .textAlign(TextAlign.Center)
        
        Circle()
          .width(12)
          .height(12)
          .fill(this.isConnected ? '#4CAF50' : '#F44336')
      }
      .width('100%')
      .justifyContent(FlexAlign.SpaceBetween)
      .padding(20)
      
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
      .padding({ left: 20, right: 20, bottom: 20 })
      
      // Chat Messages
      if (this.isConnected) {
        List() {
          ForEach(this.messages, (message: ChatMessage) => {
            ListItem() {
              Row() {
                if (message.isUser) {
                  Blank().layoutWeight(1)
                  
                  Column() {
                    Text(message.content)
                      .fontSize(16)
                      .fontColor('#FFFFFF')
                      .padding(15)
                      .backgroundColor('#2196F3')
                      .borderRadius(15)
                      .maxLines(10)
                      .textOverflow({ overflow: TextOverflow.Ellipsis })
                  }
                  .alignItems(HorizontalAlign.End)
                  .width('80%')
                } else {
                  Column() {
                    Text(message.content)
                      .fontSize(16)
                      .fontColor('#000000')
                      .padding(15)
                      .backgroundColor('#F0F0F0')
                      .borderRadius(15)
                      .maxLines(10)
                      .textOverflow({ overflow: TextOverflow.Ellipsis })
                  }
                  .alignItems(HorizontalAlign.Start)
                  .width('80%')
                  
                  Blank().layoutWeight(1)
                }
              }
              .width('100%')
              .padding({ left: 15, right: 15, top: 5, bottom: 5 })
            }
          })
        }
        .layoutWeight(1)
        .padding({ bottom: 10 })
        
        // Input Section
        Row() {
          TextInput({ 
            text: this.inputText,
            placeholder: 'Type your message...'
          })
            .layoutWeight(1)
            .onChange((value) => {
              this.inputText = value
            })
            .onSubmit(() => {
              this.sendMessage()
            })
          
          Button(this.isLoading ? 'Sending...' : 'Send')
            .width(80)
            .margin({ left: 10 })
            .backgroundColor('#4CAF50')
            .enabled(!this.isLoading && this.inputText.trim().length > 0)
            .onClick(() => {
              this.sendMessage()
            })
        }
        .width('100%')
        .padding(20)
        .backgroundColor('#FFFFFF')
        
        Button('Clear Chat')
          .width('100%')
          .margin({ top: 10, left: 20, right: 20 })
          .backgroundColor('#FF9800')
          .onClick(() => {
            this.controller.clearMessages()
            this.messages = []
          })
      }
    }
    .width('100%')
    .height('100%')
    .backgroundColor('#FAFAFA')
  }
}