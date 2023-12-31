#include <stdint.h>
#include <string.h>
#include <sys/timerfd.h>
#include <pthread.h>
#include <poll.h>
#include <stdio.h>
#include <unistd.h>

#include "timer.h"

#define MAX_TIMER_COUNT 10

struct timer_node {
    int                 fd;
    time_handler        callback;
    void *              user_data;
    unsigned int        interval;
    t_timer             type;
    struct timer_node * next;
};

static void *_timer_thread(void *data);

static pthread_t g_thread_id;
static struct timer_node *g_head = NULL;

int timer_init() {
   if (pthread_create(&g_thread_id, NULL, _timer_thread, NULL)) {
      return 0;
   }
   return 1;
}

size_t timer_start(unsigned int interval, time_handler handler, t_timer type, void *user_data) {
   struct timer_node *new_node = NULL;
   struct itimerspec new_value;

   new_node = (struct timer_node*)malloc(sizeof(struct timer_node));

   if (new_node == NULL) return 0;

   new_node->callback  = handler;
   new_node->user_data = user_data;
   new_node->interval  = interval;
   new_node->type      = type;

   new_node->fd = timerfd_create(CLOCK_REALTIME, 0);

   if (new_node->fd == -1) {
      free(new_node);
      return 0;
   }

   new_value.it_value.tv_sec = interval / 1000;
   new_value.it_value.tv_nsec = (interval % 1000)* 1000000;

   if (type == TIMER_PERIODIC) {
      new_value.it_interval.tv_sec= interval / 1000;
      new_value.it_interval.tv_nsec = (interval %1000) * 1000000;
   }
   else {
      new_value.it_interval.tv_sec= 0;
      new_value.it_interval.tv_nsec = 0;
   }

   timerfd_settime(new_node->fd, 0, &new_value, NULL);

   /*Insert the timer node into the list*/
   new_node->next = g_head;
   g_head = new_node;

   return (size_t)new_node;
}

void timer_stop(size_t timer_id) {
   struct timer_node * tn = NULL;
   struct timer_node * node = (struct timer_node *)timer_id;

   if (node == NULL) return;

   if (node == g_head) {
      g_head = g_head->next;
   }
   else {
      tn = g_head;
      while(tn && tn->next != node) tn = tn->next;
      if (tn) {
          tn->next = tn->next->next;
          close(node->fd);
          free(node);
      }
   }
}

void timer_final() {

   while(g_head) timer_stop((size_t)g_head);
   pthread_cancel(g_thread_id);
   pthread_join(g_thread_id, NULL);

}

struct timer_node *_get_timer_from_fd(int fd) {

   struct timer_node *tn = g_head;

   while (tn) {
      if (tn->fd == fd) return tn;
      tn = tn->next;
   }
   return NULL;
}

void *_timer_thread(void * data) {

    struct pollfd ufds[MAX_TIMER_COUNT] = {{0}};
    int max_count = 0;
    struct timer_node *tn = NULL;
    int read_fds = 0, i, s;
    uint64_t exp;

   while (1) {

      // Prevent arbitrary cancellation
      pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
      pthread_testcancel();
      pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);

      max_count = 0;
      tn = g_head;

      memset(ufds, 0, sizeof(struct pollfd)*MAX_TIMER_COUNT);

      while (tn) {
         ufds[max_count].fd = tn->fd;
         ufds[max_count].events = POLLIN;
         max_count++;
         tn = tn->next;
      }

      read_fds = poll(ufds, max_count, 100);

      if (read_fds <= 0) continue;

      for (i=0;i<max_count;i++) {
         if (ufds[i].revents & POLLIN) {
            s = read(ufds[i].fd, &exp, sizeof(uint64_t));
            if (s != sizeof(uint64_t)) continue;
            tn = _get_timer_from_fd(ufds[i].fd);
            if (tn && tn->callback) tn->callback((size_t)tn, tn->user_data);
         }
      }
   }

   return NULL;
}
