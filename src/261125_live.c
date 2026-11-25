#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <string.h>
#include <zmk/endpoints.h>
#include <zmk/keymap.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/event_manager.h>
#include "261125_probe.h"

#define LIVE_UUID(n) BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x7a98d900+(n),0x2641,0x4c40,0xb418,0x25a904bc1771ULL))
#define CHORD (BIT64(30)|BIT64(40))
enum { OFF, ARM, ON };
enum { NONE, SERIAL, RADIO };
static const struct device *const uart=DEVICE_DT_GET(DT_NODELABEL(linea40_probe_uart));
static struct k_spinlock keys_lock;
static uint64_t held,taps;
static bool tracking;
static K_MUTEX_DEFINE(control);
static int owner,phase;
static int64_t expires,hold_since,last_touch,last_tx;
static bool must_release;
static uint8_t previous[16];
static struct bt_conn *peer;
static void tick(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(ticker,tick);

int __real_raise_zmk_position_state_changed(struct zmk_position_state_changed event);
int __wrap_raise_zmk_position_state_changed(struct zmk_position_state_changed event){
    if(event.position<41){
        uint64_t bit=BIT64(event.position);
        k_spinlock_key_t k=k_spin_lock(&keys_lock);
        if(event.state){held|=bit;if(tracking)taps|=bit;}else{held&=~bit;}
        k_spin_unlock(&keys_lock,k);
    }
    return __real_raise_zmk_position_state_changed(event);
}
static uint64_t bitmap(uint64_t *pulse){
    k_spinlock_key_t k=k_spin_lock(&keys_lock);
    uint64_t v=held;
    if(pulse){*pulse=taps;taps=0;}
    k_spin_unlock(&keys_lock,k);return v;
}
static void track(bool enable){
    k_spinlock_key_t k=k_spin_lock(&keys_lock);
    tracking=enable;taps=0;k_spin_unlock(&keys_lock,k);
}
static bool allowed(int transport,struct bt_conn *conn){
    struct zmk_endpoint_instance endpoint=zmk_endpoints_selected();
    if(transport==SERIAL)return endpoint.transport==ZMK_TRANSPORT_USB;
    if(!conn||endpoint.transport!=ZMK_TRANSPORT_BLE||bt_conn_get_security(conn)<BT_SECURITY_L2||
       zmk_ble_profile_index(bt_conn_get_dst(conn))!=endpoint.ble.profile_index)return false;
    struct bt_conn *active=zmk_ble_active_profile_conn();
    bool same=active&&active==conn;if(active)bt_conn_unref(active);return same;
}
// All control state is serialized. No ordered events or typed text are retained.
static void stop(void){
    track(false);owner=NONE;phase=OFF;
    if(peer){bt_conn_unref(peer);peer=NULL;}
}
struct bond_query { const bt_addr_le_t *addr; bool found; };
static void match_bond(const struct bt_bond_info *b,void *user){
    struct bond_query *q=user;
    if(bt_addr_le_cmp(&b->addr,q->addr)==0)q->found=true;
}
static bool begin(int transport,struct bt_conn *conn){
    if(!allowed(transport,conn)||owner!=NONE)return false;
    if(conn){
        struct bond_query q={.addr=bt_conn_get_dst(conn)};
        bt_foreach_bond(BT_ID_DEFAULT,match_bond,&q);if(!q.found)return false;
    }
    owner=transport;phase=ARM;peer=conn?bt_conn_ref(conn):NULL;
    hold_since=0;must_release=(bitmap(NULL)&CHORD)!=0;
    last_touch=k_uptime_get();expires=last_touch+30000;last_tx=0;
    memset(previous,0xff,sizeof(previous));k_work_reschedule(&ticker,K_MSEC(25));return true;
}
static void frame(uint8_t out[16]){
    memset(out,0,16);out[0]=0x40;out[1]=4;out[2]=phase;
    if(phase!=ON)return;
    uint64_t pulse=0,keys=bitmap(&pulse);
    out[3]=zmk_keymap_layer_index_to_id(zmk_keymap_highest_layer_active());
    for(int i=0;i<6;i++){out[4+i]=keys>>(8*i);out[10+i]=pulse>>(8*i);}
}
static ssize_t command(struct bt_conn *conn,const struct bt_gatt_attr *a,const void *buf,
                       uint16_t len,uint16_t offset,uint8_t flags){
    ARG_UNUSED(a);ARG_UNUSED(flags);
    if(offset||len!=1)return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    if(!allowed(RADIO,conn))return BT_GATT_ERR(BT_ATT_ERR_AUTHORIZATION);
    uint8_t op=*(const uint8_t *)buf;
    k_mutex_lock(&control,K_FOREVER);bool ok=false;
    if(op==1)ok=begin(RADIO,conn);
    else if(op==0&&owner==RADIO&&peer==conn){stop();ok=true;}
    k_mutex_unlock(&control);return ok?len:BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
}
BT_GATT_SERVICE_DEFINE(zz_linea40_live,BT_GATT_PRIMARY_SERVICE(LIVE_UUID(0)),
    BT_GATT_CHARACTERISTIC(LIVE_UUID(1),BT_GATT_CHRC_NOTIFY,BT_GATT_PERM_NONE,NULL,NULL,NULL),
    BT_GATT_CCC(NULL,BT_GATT_PERM_READ_ENCRYPT|BT_GATT_PERM_WRITE_ENCRYPT),
    BT_GATT_CHARACTERISTIC(LIVE_UUID(2),BT_GATT_CHRC_WRITE,BT_GATT_PERM_WRITE_ENCRYPT,NULL,command,NULL));
static void tick(struct k_work *w){
    ARG_UNUSED(w);k_mutex_lock(&control,K_FOREVER);
    if(!owner)goto done;
    const struct bt_gatt_attr *a=&zz_linea40_live.attrs[2];
    int64_t now=k_uptime_get();
    if(!allowed(owner,peer)||(owner==RADIO&&!bt_gatt_is_subscribed(peer,a,BT_GATT_CCC_NOTIFY))||
       (owner==SERIAL&&now-last_touch>4000)){stop();goto done;}
    uint64_t both=bitmap(NULL)&CHORD;
    if(!both)must_release=false;
    if(!must_release&&both==CHORD){
        if(!hold_since)hold_since=now;
        if(now-hold_since>=1800){
            must_release=true;
            if(phase==ARM){phase=ON;expires=now+30*60*1000;track(true);}
            else expires=now;
        }
    }else hold_since=0;
    if(now>=expires){phase=OFF;track(false);}
    if(owner==RADIO){
        uint8_t out[16];frame(out);
        if(memcmp(out,previous,16)||now-last_tx>=2000){
            int rc=bt_gatt_notify(peer,a,out,16);
            if(!rc){memcpy(previous,out,16);last_tx=now;}
        }
    }
    if(phase==OFF){stop();goto done;}
    k_work_reschedule(&ticker,K_MSEC(25));
done:k_mutex_unlock(&control);
}
static int endpoint_event(const zmk_event_t *eh){
    ARG_UNUSED(eh);k_mutex_lock(&control,K_FOREVER);stop();k_mutex_unlock(&control);
    return ZMK_EV_EVENT_BUBBLE;
}
ZMK_LISTENER(linea40_live_output,endpoint_event);
ZMK_SUBSCRIPTION(linea40_live_output,zmk_endpoint_changed);

static atomic_t op_pending;
static void usb_reply(struct k_work *w){
    ARG_UNUSED(w);int op=atomic_set(&op_pending,0);if(!op)return;
    if(op=='I'){
        bool usb_output=allowed(SERIAL,NULL);
        uint8_t old[9];if(probe_reply(old,usb_output&&!!(bitmap(NULL)&BIT64(19)),usb_output?zmk_keymap_layer_index_to_id(zmk_keymap_highest_layer_active()):0))
            for(size_t i=0;i<sizeof(old);i++)uart_poll_out(uart,old[i]);
        return;
    }
    uint8_t out[16]={0x40,4};
    k_mutex_lock(&control,K_FOREVER);
    if(op=='A')begin(SERIAL,NULL);
    if(owner==SERIAL){
        if(op=='X'||!allowed(SERIAL,NULL))stop();
        else{last_touch=k_uptime_get();frame(out);}
    }
    k_mutex_unlock(&control);
    uart_poll_out(uart,0xab);
    for(size_t i=0;i<sizeof(out);i++){if(out[i]>=0xab&&out[i]<=0xad)uart_poll_out(uart,0xac);uart_poll_out(uart,out[i]);}
    uart_poll_out(uart,0xad);
}
static K_WORK_DEFINE(serial_work,usb_reply);
static void rx(const struct device *d,void *ctx){
    ARG_UNUSED(ctx);static uint8_t info,idx;static const uint8_t prefix[]={'R','4','?'};
    uint8_t bytes[16];if(!uart_irq_update(d)||!uart_irq_rx_ready(d))return;
    int n;while((n=uart_fifo_read(d,bytes,sizeof(bytes)))>0)for(int i=0;i<n;i++){
        uint8_t b=bytes[i],op=0;
        if(probe_request(&info,b))op='I';
        if(idx==sizeof(prefix)){if(b=='A'||b=='P'||b=='X')op=b;idx=0;}
        else if(b==prefix[idx])idx++;else idx=(b=='R');
        if(op){atomic_set(&op_pending,op);k_work_submit(&serial_work);}
    }
}
static int init_live(void){
    if(!device_is_ready(uart))return -ENODEV;
    int rc=uart_irq_callback_user_data_set(uart,rx,NULL);if(rc)return rc;
    uart_irq_rx_enable(uart);return 0;
}
SYS_INIT(init_live,APPLICATION,CONFIG_APPLICATION_INIT_PRIORITY);
