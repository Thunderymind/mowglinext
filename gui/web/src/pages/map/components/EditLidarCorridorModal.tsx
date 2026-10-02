import {Modal, InputNumber, Input, Form} from "antd";
import {useTranslation} from "react-i18next";
import type {LidarIgnoreCorridor} from "../../../types/ros.ts";

interface EditLidarCorridorModalProps {
    corridor: LidarIgnoreCorridor | null;
    busy: boolean;
    onSave: (name: string, widthM: number) => void;
    onCancel: () => void;
}

/// The one place a line's name and ignore distance are edited, on both
/// desktop and mobile — same pattern as EditAreaModal: select the line (row
/// click in the list, or on the map), then "Edit properties" in the toolbar
/// opens this. Replaces the old per-row inline width field in
/// LidarCorridorsPanel, which had nowhere to put a name input and doesn't
/// exist on mobile at all. Position/points are still edited by dragging on
/// the map (same DrawControl session as desktop) — only name + width need a
/// control that isn't a drag gesture.
export const EditLidarCorridorModal = ({corridor, ...rest}: EditLidarCorridorModalProps) => {
    if (!corridor) return null;
    // The form instance lives in the inner component: antd applies
    // `initialValues` only once per instance, so a form that outlived the
    // modal kept showing the first line's name/width for every later line —
    // and Save then wrote those stale values onto the line being edited.
    return <EditLidarCorridorForm key={corridor.id ?? corridor.name ?? ''} corridor={corridor} {...rest}/>;
};

const EditLidarCorridorForm = ({corridor, busy, onSave, onCancel}: EditLidarCorridorModalProps & {corridor: LidarIgnoreCorridor}) => {
    const {t} = useTranslation();
    const [form] = Form.useForm<{name: string; widthCm: number}>();

    return (
        <Modal
            open
            title={corridor.name || t('mapLidarCorridors.unnamed', {id: corridor.id ?? ''})}
            okText={t('mapLidarCorridors.save')}
            cancelText={t('mapLidarCorridors.cancel')}
            confirmLoading={busy}
            onCancel={onCancel}
            onOk={() => {
                const {name, widthCm} = form.getFieldsValue();
                onSave(name.trim(), widthCm / 100);
            }}
        >
            <Form form={form} layout="vertical" initialValues={{name: corridor.name ?? '', widthCm: Math.round((corridor.width_m ?? 0.2) * 100)}}>
                <Form.Item name="name" label={t('mapLidarCorridors.nameLabel')}>
                    <Input placeholder={t('mapLidarCorridors.namePlaceholder')} autoFocus maxLength={64}/>
                </Form.Item>
                <Form.Item name="widthCm" label={t('mapLidarCorridors.widthLabel')}
                    tooltip={t('mapLidarCorridors.widthTooltip')}>
                    <InputNumber min={5} max={100} step={5} precision={0} addonAfter="cm" style={{width: '100%'}}/>
                </Form.Item>
            </Form>
        </Modal>
    );
};
