# Libraries

This folder is for project libraries that are not tied to STM32 HAL, CMSIS, or a
specific board.

Suggested structure:

- `Inc/` for public headers
- `Src/` for implementation files

Recommended layering:

- Keep reusable algorithms here
- Keep HAL, GPIO, timer, UART, CAN, and board-specific code in `Core`
- Connect both layers with a thin adapter in the application code

Example:

```text
Libraries/
  MotorControl/
    Inc/
    Src/
```
