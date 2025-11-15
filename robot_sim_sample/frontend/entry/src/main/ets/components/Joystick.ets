@Component
export struct JoystickComponent {
  @State private centerX: number = 100
  @State private centerY: number = 100
  @State private knobX: number = 100
  @State private knobY: number = 100
  @State private isDragging: boolean = false
  @State private joystickRadius: number = 80
  @State private knobRadius: number = 25
  
  onJoystickMove: (x: number, y: number) => void = () => {}
  onJoystickRelease: () => void = () => {}
  
  build() {
    Stack() {
      // Outer circle (joystick base)
      Circle()
        .width(this.joystickRadius * 2)
        .height(this.joystickRadius * 2)
        .fill('#E0E0E0')
        .stroke('#CCCCCC')
        .strokeWidth(2)
      
      // Inner circle (knob)
      Circle()
        .width(this.knobRadius * 2)
        .height(this.knobRadius * 2)
        .fill(this.isDragging ? '#1976D2' : '#2196F3')
        .position({
          x: this.knobX - this.knobRadius,
          y: this.knobY - this.knobRadius
        })
        .gesture(
          PanGesture()
            .onActionStart((event) => {
              this.isDragging = true
            })
            .onActionUpdate((event) => {
              // Calculate new knob position
              let newX = this.centerX + event.offsetX
              let newY = this.centerY + event.offsetY
              
              // Constrain knob to joystick circle
              let dx = newX - this.centerX
              let dy = newY - this.centerY
              let distance = Math.sqrt(dx * dx + dy * dy)
              
              if (distance <= this.joystickRadius - this.knobRadius) {
                this.knobX = newX
                this.knobY = newY
              } else {
                // Clamp to edge
                let angle = Math.atan2(dy, dx)
                let maxDistance = this.joystickRadius - this.knobRadius
                this.knobX = this.centerX + Math.cos(angle) * maxDistance
                this.knobY = this.centerY + Math.sin(angle) * maxDistance
              }
              
              // Calculate normalized values (-1 to 1)
              let normalizedX = (this.knobX - this.centerX) / (this.joystickRadius - this.knobRadius)
              let normalizedY = -(this.knobY - this.centerY) / (this.joystickRadius - this.knobRadius) // Invert Y
              
              this.onJoystickMove(normalizedX, normalizedY)
            })
            .onActionEnd((event) => {
              this.isDragging = false
              // Return knob to center
              this.knobX = this.centerX
              this.knobY = this.centerY
              this.onJoystickRelease()
            })
        )
      
      // Center dot
      Circle()
        .width(4)
        .height(4)
        .fill('#666666')
    }
    .width(this.joystickRadius * 2)
    .height(this.joystickRadius * 2)
    .onAreaChange((oldValue, newValue) => {
      // Update center position when component size changes
      this.centerX = Number(newValue.width) / 2
      this.centerY = Number(newValue.height) / 2
      this.knobX = this.centerX
      this.knobY = this.centerY
    })
  }

}