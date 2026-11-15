import { defineComponent, ref, shallowRef, type PropType } from "vue";
import { useEventListener } from "@vueuse/core";
import style from "./style.module.scss";

export type MessageType = "info" | "success" | "warning" | "error";

export interface MessageOptions {
  type?: MessageType;
  duration?: number;
}

interface MessageItem {
  id: number;
  content: string;
  type: MessageType;
}

const messages = ref<MessageItem[]>([]);
let nextMessageId = 0;

export function dismissMessage(id: number) {
  messages.value = messages.value.filter((message) => message.id !== id);
}

export function showMessage(content: string, options: MessageOptions = {}) {
  const id = ++nextMessageId;
  messages.value.push({
    id,
    content,
    type: options.type ?? "info",
  });
  if ((options.duration ?? 4_000) > 0) {
    window.setTimeout(() => dismissMessage(id), options.duration ?? 4_000);
  }
  return () => dismissMessage(id);
}

const MessageItem = defineComponent({
  name: "MessageItem",
  props: {
    message: { type: Object as PropType<MessageItem>, required: true },
  },
  setup(props) {
    const closeButton = shallowRef<HTMLButtonElement>();
    useEventListener(closeButton, "click", () => dismissMessage(props.message.id));

    return () => (
      <div
        class={[style.message, style[props.message.type]]}
        data-message-type={props.message.type}
        role="alert"
      >
        <span>{props.message.content}</span>
        <button ref={closeButton} type="button" class={style.close} aria-label="关闭消息">
          ×
        </button>
      </div>
    );
  },
});

export const Message = defineComponent(() => () => (
  <div class={style.messageHost} aria-live="polite" aria-atomic="false">
    {messages.value.map((message) => (
      <MessageItem key={message.id} message={message} />
    ))}
  </div>
));
