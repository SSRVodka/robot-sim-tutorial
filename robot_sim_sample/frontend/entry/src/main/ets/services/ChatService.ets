import http from '@ohos.net.http';
import logger from '../model/Logger';

export interface ChatMessage {
  message: string
  isUser: boolean
  timestamp: number
}

export interface SendMessageResponse {
  id: string
  message: string
}

export class ChatService {
  private baseUrl: string = 'http://192.168.144.1:8000'
  
  constructor(baseUrl?: string) {
    if (baseUrl) {
      this.baseUrl = baseUrl
    }
  }
  
  async sendMessage(message: string): Promise<SendMessageResponse | null> {
    const httpRequest = http.createHttp()
    
    try {
      const response = await httpRequest.request(`${this.baseUrl}/api/demo`, {
        method: http.RequestMethod.POST,
        header: {
          'Content-Type': 'application/json'
        },
        extraData: JSON.stringify({ message })
      })
      
      if (response.responseCode === 200) {
        const result = JSON.parse(response.result as string) as SendMessageResponse
        return result
      } else {
        logger.error('Failed to send message: %{public}d', String(response.responseCode))
        return null
      }
    } catch (error) {
      logger.error('Error sending message: %{public}s', error)
      return null
    } finally {
      httpRequest.destroy()
    }
  }
  
  async getMessages(id: string): Promise<ChatMessage[]> {
    const httpRequest = http.createHttp()
    
    try {
      const response = await httpRequest.request(`${this.baseUrl}/api/msgs?id=${id}`, {
        method: http.RequestMethod.GET,
        header: {
          'Content-Type': 'application/json'
        }
      })
      
      if (response.responseCode === 200) {
        const result = JSON.parse(response.result as string) as SendMessageResponse[]
        return result.map(item => ({
          message: item.message,
          isUser: false,
          timestamp: Date.now()
        } as ChatMessage))
      } else {
        logger.error('Failed to get messages: %{public}s', String(response.responseCode))
        return []
      }
    } catch (error) {
      logger.error('Error getting messages: %{public}s', error)
      return []
    } finally {
      httpRequest.destroy()
    }
  }
}