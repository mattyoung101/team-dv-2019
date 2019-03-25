#include "music.h"

static void music_play_task(void *pvParameter){
    static const char *TAG = "MusicPlayer";
    music_note_t *song = (music_note_t*) pvParameter;
    size_t size = sizeof(song) / sizeof(music_note_t);

    for (int i = 0; i < size; i++){
        music_note_t note = song[i];
        ESP_LOGI(TAG, "Note: onoff(%d), freq(%d), time(%d)", note.type, note.frequency, note.time);
        vTaskDelay(pdMS_TO_TICKS(note.time));
    }
}

void music_play(music_note_t *song){
    xTaskCreate(music_play_task, "MusicPlayTask", 4096, song, configMAX_PRIORITIES - 4, NULL);
}