import { defineComponent, ref, shallowRef, type PropType } from "vue";
import { useEventListener } from "@vueuse/core";
import style from "./style.module.scss";

export type MessageType = "info" | "success" | "warning" | "error";

export interface MessageOptions {
  type?: MessageType;
  duration?: number;
  action?: {
    label: string;
    onClick: () => void;
  };
  dismissible?: boolean;
}

interface MessageItem {
  id: number;
  content: string;
  type: MessageType;
  action?: MessageOptions["action"];
  dismissible: boolean;
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
    action: options.action,
    dismissible: options.dismissible ?? true,
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
        {props.message.action && (
          <button
            type="button"
            class={style.action}
            onClick={() => {
              props.message.action?.onClick();
              dismissMessage(props.message.id);
            }}
          >
            {props.message.action.label}
          </button>
        )}
        {props.message.dismissible && (
          <button ref={closeButton} type="button" class={style.close} aria-label="关闭消息">
            ×
          </button>
        )}
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
