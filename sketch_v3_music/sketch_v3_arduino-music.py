import serial
import spotipy
from spotipy.oauth2 import SpotifyOAuth

from utils import get_all_playlist_tracks, choose_track, set_volume, get_curr_info, \
                    resume_prev_song


# connect to spotify API and set up

# connect to spotify API - if this gives issues, remember to clear the cache
# declare the scope such that the API can change the songs that a user is playing
sp = spotipy.Spotify(auth_manager=SpotifyOAuth(client_id="eed2da3210184052949d749f3621d606",
                                               client_secret="xxxxxx",
                                               redirect_uri="http://127.0.0.1:8888",
                                               scope = "user-modify-playback-state user-read-playback-state playlist-read-private playlist-read-collaborative"))
 # got a new client secret, since realized this repo was public 🤦, right after uploading everything for the final project submission

# get all the playlist tracks
tracks = get_all_playlist_tracks(sp)

# determine info about the user's current song
prev_song = {}


#  set up the connection to the Arduino
ser = serial.Serial('/dev/cu.usbmodem2101', 9600, timeout=1) # make sure to change based on your own device

ser.write(b'1')

# values is a list that will contain the values inputted by the user
# assumed that index 0 is the speed, and index 1 is the duration
values = []

try:
    # continuously loop, checking the output from the Arduino
    while True:
        if ser.in_waiting > 0:
            message = ser.readline().decode('utf-8').strip()
            print(message)

            # to use the same Arduino code as in V2, we check for specific outputs
            # from the Arduino, and if we receive those inputs, we ask the 
            # user to input the next value
            if 'enter' in message:
                value = input()
                ser.write(f"{value}".encode())
                if len(values) < 2:
                    values.append(int(value))

                if len(values) == 2:
                    track = choose_track(values, tracks)

            if 'Starting' in message:
                print("Playing song now :)")
                print(track)
                prev_song = get_curr_info(sp)
                sp.start_playback(uris=[f'spotify:track:{track}'])

                set_volume(sp, values)

            if 'done' in message:
                if prev_song == None:
                    print("No song was playing before, so not resuming anything.")
                else:
                    resume_prev_song(sp, prev_song)
                    break
except KeyboardInterrupt:
    """We need to have a method to emergency stop the centrifuge now that we're using the
    Python interface. Decided to keep this simple to interface with the old V2 code,
    so we're just checking for a keyboard interrupt, and if we catch it, we just output
    and 's' to the Arduino, since that was the old stop command! STOP!"""

    # still need to resume the user's music
    if prev_song == None:
        print("No song was playing before, so not resuming anything.")
    else:
        resume_prev_song(sp, prev_song)
    ser.write("s".encode())
    print("Exiting program...")

print("Wahoo! Hopefully your original music is now playing again :)")