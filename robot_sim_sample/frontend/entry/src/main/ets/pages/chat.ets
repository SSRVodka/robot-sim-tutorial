
import libAiChatGrpc from 'libentry.so';

import logger from '../model/Logger';

export interface ChatMessage {
  id: string
  content: string
  isUser: boolean
  timestamp: Date
}

export class AiChatController {
  private isConnected: boolean = false
  private serverAddress: string = 'localhost:50051'
  private messages: ChatMessage[] = []
  
  constructor() {
    logger.info('AiChatController initialized')
  }
  
  async connect(address?: string): Promise<boolean> {
    if (address) {
      this.serverAddress = address
    }
    
    try {
      this.isConnected = libAiChatGrpc.connectChat(this.serverAddress)
      logger.info(`AI Chat connection ${this.isConnected ? 'successful' : 'failed'} to ${this.serverAddress}`)
      return this.isConnected
    } catch (error) {
      logger.error('Failed to connect to AI chat server:', error)
      this.isConnected = false
      return false
    }
  }
  
  async sendMessage(content: string): Promise<string | null> {
    if (!this.isConnected) {
      logger.warn('Not connected to AI chat server')
      return null
    }
    
    try {
      const userMessage: ChatMessage = {
        id: Date.now().toString(),
        content,
        isUser: true,
        timestamp: new Date()
      }
      this.messages.push(userMessage)
      
      const response: string = await libAiChatGrpc.sendChatMessage(content)
      
      if (response) {
        const aiMessage: ChatMessage = {
          id: (Date.now() + 1).toString(),
          content: response,
          isUser: false,
          timestamp: new Date()
        }
        this.messages.push(aiMessage)
      }
      
      return response
    } catch (error) {
      logger.error('Failed to send chat message:', error)
      return null
    }
  }
  
  getMessages(): ChatMessage[] {
    return this.messages
  }
  
  clearMessages(): void {
    this.messages = []
  }
  
  getConnectionStatus(): boolean {
    return this.isConnected
  }
  
  getServerAddress(): string {
    return this.serverAddress
  }
}