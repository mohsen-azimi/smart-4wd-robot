from pynput import keyboard


def on_press(key):
    try:
        # print('alphanumeric key {0} pressed'.format(
        #     key.char))
        print(key)
    except AttributeError:

        # print('special key {0} pressed'.format(
        #     key))
        print(key)

def on_release(key):
    print("key", key)
    print('{0} released'.format(
        key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False


# Collect events until released
with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()

