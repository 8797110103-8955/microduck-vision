import { defineComponent } from "vue";
import style from "./style.module.scss";

export const actionKind = (value: string) =>
  value === "sequence" || value === "policy" ? value : "pose";

export const servoModel = (value: string) => (value === "xl330" ? "xl330" : "hd1910");

export const SectionHeading = defineComponent(
  (props: { eyebrow: string; title: string; text?: string }) => () => (
    <div class={style.sectionHeading}>
      <div>
        <span class={style.eyebrow}>{props.eyebrow}</span>
        <h1>{props.title}</h1>
        {props.text && <p class={style.muted}>{props.text}</p>}
      </div>
    </div>
  ),
  { props: ["eyebrow", "title", "text"] },
);
