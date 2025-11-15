import { ChatService, ChatMessage } from '../services/ChatService';
import router from '@ohos.router';

import logger from '../model/Logger';

@Entry
@Component
struct ChatPage {
  @State private chatService: ChatService = new ChatService()
  @State private messages: ChatMessage[] = []
  @State private inputText: string = ''
  @State private isLoading: boolean = false
  @State private serverUrl: string = 'http://192.168.144.1:8000'
  
  private scroller: Scroller = new Scroller()
  private pollingTimer: number | undefined = undefined
  private currentChatId: string | undefined = undefined
  
  aboutToAppear() {
    logger.info('ChatPage aboutToAppear')
  }
  
  aboutToDisappear() {
    if (this.pollingTimer) {
      clearInterval(this.pollingTimer)
    }
  }
  
  private async sendMessage() {
    if (!this.inputText.trim() || this.isLoading) {
      return
    }
    
    const userMessage: ChatMessage = {
      message: this.inputText,
      isUser: true,
      timestamp: Date.now()
    }
    
    this.messages.push(userMessage)
    const messageToSend = this.inputText
    this.inputText = ''
    this.isLoading = true
    
    // Scroll to bottom
    setTimeout(() => {
      this.scroller.scrollToIndex(this.messages.length - 1)
    }, 50)
    
    try {
      const response = await this.chatService.sendMessage(messageToSend)
      
      if (response) {
        this.currentChatId = response.id
        
        // Start polling for replies
        this.startPolling()
        
        // Add server confirmation message
        const confirmMessage: ChatMessage = {
          message: response.message,
          isUser: false,
          timestamp: Date.now()
        }
        this.messages.push(confirmMessage)
        
        setTimeout(() => {
          this.scroller.scrollToIndex(this.messages.length - 1)
        }, 50)
      }
    } catch (error) {
      logger.error('Failed to send message:', error)
    } finally {
      this.isLoading = false
    }
  }
  
  private startPolling() {
    if (this.pollingTimer) {
      clearInterval(this.pollingTimer)
    }
    
    this.pollingTimer = setInterval(async () => {
      if (this.currentChatId) {
        await this.fetchReplies()
      }
    }, 2000) // Poll every 2 seconds
  }
  
  private async fetchReplies() {
    if (!this.currentChatId) return
    
    try {
      const replies = await this.chatService.getMessages(this.currentChatId)
      
      if (replies.length > 0) {
        // Add new replies to messages
        replies.forEach(reply => {
          this.messages.push(reply)
        })
        
        // Stop polling after receiving replies
        if (this.pollingTimer) {
          clearInterval(this.pollingTimer)
          this.pollingTimer = undefined
        }
        
        setTimeout(() => {
          this.scroller.scrollToIndex(this.messages.length - 1)
        }, 50)
      }
    } catch (error) {
      logger.error('Failed to fetch replies:', error)
    }
  }
  
  private navigateToJoystick() {
    router.pushUrl({
      url: 'pages/JoystickPage'
    })
  }
  
  build() {
    Column() {
      // Header with navigation
      Row() {
        Button('← Joystick')
          .backgroundColor('#2196F3')
          .fontSize(16)
          .onClick(() => {
            this.navigateToJoystick()
          })
        
        Text('AI Chat')
          .fontSize(24)
          .fontWeight(FontWeight.Bold)
          .flexGrow(1)
          .textAlign(TextAlign.Center)
        
        // Server URL setting
        Button('⚙')
          .backgroundColor('#757575')
          .fontSize(16)
          .onClick(() => {
            // Could show settings dialog
          })
      }
      .width('100%')
      .padding({ left: 20, right: 20, top: 20, bottom: 10 })
      .justifyContent(FlexAlign.SpaceBetween)
      
      // Server URL input
      Row() {
        Text('Server:')
          .fontSize(14)
          .margin({ right: 10 })
        
        TextInput({ text: this.serverUrl })
          .fontSize(14)
          .flexGrow(1)
          .onChange((value) => {
            this.serverUrl = value
            this.chatService = new ChatService(value)
          })
      }
      .width('100%')
      .padding({ left: 20, right: 20, bottom: 10 })
      
      Divider().color('#E0E0E0')
      
      // Messages list
      List({ scroller: this.scroller }) {
        ForEach(this.messages, (message: ChatMessage, index: number) => {
          ListItem() {
            Row() {
              if (message.isUser) {
                Blank().layoutWeight(1)
                
                Text(message.message)
                  .fontSize(16)
                  .fontColor('#FFFFFF')
                  .backgroundColor('#2196F3')
                  .padding(12)
                  .borderRadius(8)
                //   .maxWidth('80%')
                  .textAlign(TextAlign.Start)
              } else {
                Text(message.message)
                  .fontSize(16)
                  .fontColor('#000000')
                  .backgroundColor('#F0F0F0')
                  .padding(12)
                  .borderRadius(8)
                //   .maxWidth('80%')
                  .textAlign(TextAlign.Start)
                
                Blank().layoutWeight(1)
              }
            }
            .width('100%')
            .margin({ top: 8, bottom: 8 })
          }
        }, (message: ChatMessage, index: number) => `${index}-${message.timestamp}`)
      }
      .width('100%')
      .layoutWeight(1)
      .padding({ left: 20, right: 20 })
      .scrollBar(BarState.Off)
      
      // Loading indicator
      if (this.isLoading) {
        Row() {
          LoadingProgress()
            .width(20)
            .height(20)
            .margin({ right: 8 })
          
          Text('AI is thinking...')
            .fontSize(14)
            .fontColor('#666666')
        }
        .padding(10)
        .justifyContent(FlexAlign.Center)
      }
      
      Divider().color('#E0E0E0')
      
      // Input area
      Row() {
        TextInput({ placeholder: 'Type your message...', text: this.inputText })
          .layoutWeight(1)
          .maxLines(3)
          .borderRadius(20)
          .padding({ left: 16, right: 16 })
          .onChange((value) => {
            this.inputText = value
          })
          .onSubmit(() => {
            this.sendMessage()
          })
        
        Button('Send')
          .margin({ left: 10 })
          .backgroundColor(this.inputText.trim() && !this.isLoading ? '#2196F3' : '#CCCCCC')
          .fontColor('#FFFFFF')
          .borderRadius(20)
          .enabled(!!this.inputText.trim() && !this.isLoading)
          .onClick(() => {
            this.sendMessage()
          })
      }
      .width('100%')
      .padding(20)
      .backgroundColor('#FFFFFF')
    }
    .width('100%')
    .height('100%')
    .backgroundColor('#FAFAFA')
  }
}