import numpy as np
import spotipy
from spotipy.oauth2 import SpotifyOAuth

import numpy as np

def get_all_playlist_tracks(sp, playlist_id='6JFBHiU8hdd0xAhZ9ZyQMR'):
    """
    This function returns all the tracks in a playlist as a dictionary.
    """
    tracks = []
    results = sp.playlist_tracks(playlist_id)
    tracks.extend(results['items'])

    # Handle pagination
    while results['next']:
        results = sp.next(results)
        tracks.extend(results['items'])

    return tracks


def choose_track(values, tracks):
    """
    Returns the ID for the track that is closest in duration to the desired 
    centrifuge duration specified by the user. When there are songs of equal distance, 
    """
    desired_speed = values[0]
    desired_duration = values[1]
    # tracks = get_all_playlist_tracks(sp, playlist_id)

    min_dist = float('inf')
    track = ''

    for item in tracks:
        duration_ms = item['item']['duration_ms']
        duration_sec = duration_ms / 1000

        curr_dist = np.abs(duration_sec - desired_duration)
        if curr_dist < min_dist:
            min_dist = curr_dist
            track = item['item']['id']
        if curr_dist == min_dist:
            rand = np.random.randint(2)
            if rand:
                min_dist = curr_dist
                track = item['item']['id']

    return track

def set_volume(sp, values):
    """
    Chooses the volume of the song, based on the speed of centrifugation
    """
    desired_speed = values[0]
    desired_speed = min(desired_speed, 1500)
    desired_speed = max(desired_speed, 0)

    volume = (desired_speed / 1500) * 100

    print(desired_speed)

    print(volume)

    sp.volume(int(volume))

    playback = sp.current_playback()

    if playback and playback["device"]:
        volume = playback["device"]["volume_percent"]
        print("Current volume:", volume)

    return volume


def get_curr_info(sp):
    """
    Returns the info about the track that the user was playing before the 
    centrifuge started.
    """
    current_track = sp.current_playback()

    if current_track and current_track["device"]:
        data = {}
        data['id'] = current_track['item']['id']
        data['progress'] = current_track['progress_ms']

        print(data['id'])

        return data

    
    return None

def resume_prev_song(sp, data):
    """Resumes the song specified in data. See `get_curr_info` for the format
    of data
    """
    sp.start_playback(uris=[f'spotify:track:{data['id']}'], position_ms=data['progress'])
